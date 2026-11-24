/*
 * Web page and API.
 *
 *   GET  /                 the page (page.html, built into the firmware)
 *   GET  /api/state        JSON: firmware, Wi-Fi, CAN diagnostics, BMS
 *   GET  /api/can          text: id table, then the newest frames
 *   POST /api/ctrl?chg=0|1&dsg=0|1&bal=0|1   one or more; dsg=0 needs &sure=1
 *   POST /api/send?id=18F0F428&data=0400000100000000[&ext=1]
 *   POST /api/rate?kbps=250
 *   POST /api/clear        empties the id table and frame log
 *   POST /api/ota          body = build/jk_bms_can.bin
 *
 * Changes are POST so a browser's prefetch or a crawler can never switch
 * the pack off.
 */
#include "webserver.h"
#include "can.h"
#include "jkbms.h"
#include "ota.h"
#include "wifi.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "web";

extern const char page_html_start[] asm("_binary_page_html_start");
extern const char page_html_end[]   asm("_binary_page_html_end");

static bool get_param(httpd_req_t *req, const char *key, char *out, size_t out_len)
{
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    return httpd_query_key_value(query, key, out, out_len) == ESP_OK;
}

/* -1 if absent, else 0 or 1. -2 if present but not 0/1. */
static int get_bit(httpd_req_t *req, const char *key)
{
    char v[4];
    if (!get_param(req, key, v, sizeof(v))) {
        return -1;
    }
    if (strcmp(v, "0") == 0) return 0;
    if (strcmp(v, "1") == 0) return 1;
    return -2;
}

static esp_err_t reply_text(httpd_req_t *req, const char *body)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t reply_err(httpd_req_t *req, const char *status, const char *body)
{
    httpd_resp_set_status(req, status);
    return reply_text(req, body);
}

static esp_err_t h_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    /* EMBED_TXTFILES adds a trailing NUL; do not send it. */
    return httpd_resp_send(req, page_html_start, page_html_end - page_html_start - 1);
}

static esp_err_t h_state(httpd_req_t *req)
{
    /* Static: the server runs one handler at a time, and this is too big
     * for the handler's stack. */
    static char buf[6144];
    can_diag_t d;
    can_get_diag(&d);

    size_t n = snprintf(buf, sizeof(buf),
        "{\"fw\":\"%s\",\"up_s\":%lu,\"heap\":%lu,"
        "\"wifi\":{\"up\":%s,\"ip\":\"%s\",\"rssi\":%d,\"ap\":%s},"
        "\"can\":{\"kbps\":%lu,\"state\":\"%s\",\"tec\":%lu,\"rec\":%lu,\"rx\":%lu,\"rate\":%lu,"
        "\"tx_ok\":%lu,\"tx_fail\":%lu,\"bit\":%lu,\"ack\":%lu,\"stuff\":%lu,\"form\":%lu,"
        "\"busoff\":%lu,\"rxpin\":%u},\"bms\":",
        ota_running_version(), (unsigned long)(esp_timer_get_time() / 1000000),
        (unsigned long)esp_get_free_heap_size(),
        wifi_sta_up() ? "true" : "false", wifi_sta_ip(), wifi_rssi(),
        wifi_ap_on() ? "true" : "false",
        (unsigned long)d.kbps, can_state_str(d.state), (unsigned long)d.tec,
        (unsigned long)d.rec, (unsigned long)d.rx, (unsigned long)d.rate,
        (unsigned long)d.tx_ok, (unsigned long)d.tx_fail, (unsigned long)d.err_bit,
        (unsigned long)d.err_ack, (unsigned long)d.err_stuff, (unsigned long)d.err_form,
        (unsigned long)d.bus_off_recoveries, d.rxpin_low_pm);
    if (n < sizeof(buf) - 2) {
        n += jk_json(buf + n, sizeof(buf) - n - 2);
    }
    if (n >= sizeof(buf) - 2) {
        return reply_err(req, "500 Internal Server Error", "state too big for the buffer");
    }
    buf[n++] = '}';
    buf[n] = '\0';

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t h_can(httpd_req_t *req)
{
    static char buf[8192];
    size_t n = can_ids_text(buf, sizeof(buf) / 2);
    n += snprintf(buf + n, sizeof(buf) - n, "\nNEWEST FRAMES (TX = sent by this board)\n");
    can_log_text(buf + n, sizeof(buf) - n);
    return reply_text(req, buf);
}

static esp_err_t h_ctrl(httpd_req_t *req)
{
    int chg = get_bit(req, "chg");
    int dsg = get_bit(req, "dsg");
    int bal = get_bit(req, "bal");
    if (chg == -2 || dsg == -2 || bal == -2) {
        return reply_err(req, "400 Bad Request", "values must be 0 or 1");
    }
    if (chg < 0 && dsg < 0 && bal < 0) {
        return reply_err(req, "400 Bad Request", "give chg, dsg or bal");
    }
    /* Discharge off cuts the pack output: the bike turns off. */
    if (dsg == 0 && get_bit(req, "sure") != 1) {
        return reply_err(req, "400 Bad Request",
                         "dsg=0 turns the pack output off. Add &sure=1 to do it.");
    }
    esp_err_t err = jk_control(chg, dsg, bal);
    if (err != ESP_OK) {
        return reply_err(req, "502 Bad Gateway", jk_ctrl_status());
    }
    return reply_text(req, jk_ctrl_status());
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static esp_err_t h_send(httpd_req_t *req)
{
    char idtxt[12] = "", datatxt[40] = "", exttxt[4] = "";
    if (!get_param(req, "id", idtxt, sizeof(idtxt)) || !idtxt[0]) {
        return reply_err(req, "400 Bad Request", "give id= in hex, e.g. id=18F0F428");
    }
    get_param(req, "data", datatxt, sizeof(datatxt));
    get_param(req, "ext", exttxt, sizeof(exttxt));

    char *end = NULL;
    unsigned long id = strtoul(idtxt, &end, 16);
    if (!end || *end != '\0' || id > 0x1FFFFFFF) {
        return reply_err(req, "400 Bad Request", "id is not valid hex (max 1FFFFFFF)");
    }
    /* More than 3 digits, or above 0x7FF, means extended — the same rule as
     * the id table, where extended ids print as 8 digits. */
    bool ext = exttxt[0] == '1' || strlen(idtxt) > 3 || id > 0x7FF;

    uint8_t data[8];
    int len = 0, nib = -1;
    for (const char *p = datatxt; *p; p++) {
        if (*p == ' ' || *p == ':' || *p == '-' || *p == '+') {
            continue;
        }
        int v = hexval(*p);
        if (v < 0) {
            return reply_err(req, "400 Bad Request", "data is not hex");
        }
        if (nib < 0) {
            nib = v;
        } else {
            if (len == 8) {
                return reply_err(req, "400 Bad Request", "data is more than 8 bytes");
            }
            data[len++] = (uint8_t)(nib << 4 | v);
            nib = -1;
        }
    }
    if (nib >= 0) {
        return reply_err(req, "400 Bad Request", "data has an odd number of hex digits");
    }

    esp_err_t err = can_send((uint32_t)id, ext, data, (uint8_t)len);
    char out[96];
    snprintf(out, sizeof(out), "%s %s id %lX, %d bytes%s", err == ESP_OK ? "sent" : "FAILED:",
             ext ? "extended" : "standard", id, len,
             err == ESP_OK ? "" : " - no node ACKed it, or the bus is down");
    if (err != ESP_OK) {
        return reply_err(req, "502 Bad Gateway", out);
    }
    return reply_text(req, out);
}

static esp_err_t h_rate(httpd_req_t *req)
{
    char v[8] = "";
    get_param(req, "kbps", v, sizeof(v));
    esp_err_t err = can_set_kbps((uint32_t)atoi(v));
    if (err == ESP_ERR_INVALID_ARG) {
        return reply_err(req, "400 Bad Request", "kbps must be 125, 250, 500 or 1000");
    }
    if (err != ESP_OK) {
        return reply_err(req, "500 Internal Server Error", esp_err_to_name(err));
    }
    char out[40];
    snprintf(out, sizeof(out), "now %lu kbit/s", (unsigned long)can_kbps());
    return reply_text(req, out);
}

static esp_err_t h_clear(httpd_req_t *req)
{
    can_ids_clear();
    return reply_text(req, "cleared");
}

static esp_err_t h_ota(httpd_req_t *req)
{
    if (CONFIG_JKB_OTA_TOKEN[0]) {
        char tok[65] = "";
        if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", tok, sizeof(tok)) != ESP_OK) {
            get_param(req, "token", tok, sizeof(tok));
        }
        if (strcmp(tok, CONFIG_JKB_OTA_TOKEN) != 0) {
            return reply_err(req, "403 Forbidden", "wrong or missing OTA token");
        }
    }
    size_t total = req->content_len;
    if (total == 0) {
        return reply_err(req, "400 Bad Request", "body must be the firmware .bin");
    }
    if (ota_begin(total) != ESP_OK) {
        return reply_err(req, "409 Conflict", ota_error());
    }

    static uint8_t buf[1460];
    size_t got = 0, next_report = 0;
    int stalls = 0;
    while (got < total) {
        size_t want = total - got;
        if (want > sizeof(buf)) {
            want = sizeof(buf);
        }
        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++stalls < 6) {
                continue;
            }
            ota_abort();
            return reply_err(req, "408 Request Timeout", "upload stalled - old firmware kept");
        }
        if (r <= 0) {
            ota_abort();
            return reply_err(req, "400 Bad Request", "upload cut short - old firmware kept");
        }
        stalls = 0;
        if (ota_write(buf, r) != ESP_OK) {
            return reply_err(req, "400 Bad Request", ota_error());
        }
        got += r;
        if (got >= next_report) {
            next_report = got + 256 * 1024;
            ESP_LOGI(TAG, "firmware %u/%u (%u%%)", (unsigned)got, (unsigned)total,
                     (unsigned)(got * 100 / total));
        }
    }

    if (ota_finish() != ESP_OK) {
        return reply_err(req, "400 Bad Request", ota_error());
    }
    char out[128];
    snprintf(out, sizeof(out), "firmware %s written - rebooting. It is on trial for 30 s.",
             ota_new_version());
    reply_text(req, out);
    ota_restart_soon();
    return ESP_OK;
}

void webserver_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = CONFIG_JKB_HTTP_PORT;
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 12;
    cfg.max_open_sockets = 10;
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 15;

    /* A browser that reloads leaves half-sent replies behind, and each one
     * logs a warning. The requests themselves are fine. */
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return;
    }

    static const httpd_uri_t routes[] = {
        {.uri = "/",          .method = HTTP_GET,  .handler = h_root},
        {.uri = "/api/state", .method = HTTP_GET,  .handler = h_state},
        {.uri = "/api/can",   .method = HTTP_GET,  .handler = h_can},
        {.uri = "/api/ctrl",  .method = HTTP_POST, .handler = h_ctrl},
        {.uri = "/api/send",  .method = HTTP_POST, .handler = h_send},
        {.uri = "/api/rate",  .method = HTTP_POST, .handler = h_rate},
        {.uri = "/api/clear", .method = HTTP_POST, .handler = h_clear},
        {.uri = "/api/ota",   .method = HTTP_POST, .handler = h_ota},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }
    ESP_LOGI(TAG, "http server on port %d", CONFIG_JKB_HTTP_PORT);
}
