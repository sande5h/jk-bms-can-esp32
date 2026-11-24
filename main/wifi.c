#include "wifi.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"

static const char *TAG = "wifi";

static char s_ip[16] = "-";
static volatile bool s_up;
static bool s_ap;
static bool s_sta;
static int s_fails;
static esp_timer_handle_t s_retry;

/* 2, 5, 10, 30 s, then 30 s for ever. The access point shares the radio,
 * and a station that scans all the time makes the AP hard to use. */
static uint32_t backoff_ms(int fails)
{
    static const uint32_t steps[] = {2000, 5000, 10000, 30000};
    int i = fails < 4 ? fails : 3;
    return steps[i];
}

static void retry_cb(void *arg)
{
    (void)arg;
    if (!s_up) {
        esp_wifi_connect();
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)data;
        bool was_up = s_up;
        s_up = false;
        strcpy(s_ip, "-");
        uint32_t wait = backoff_ms(s_fails);
        s_fails++;
        if (was_up || s_fails <= 5 || s_fails % 10 == 0) {
            ESP_LOGW(TAG, "not joined to \"%s\" (reason %d), retry in %lu s",
                     CONFIG_JKB_WIFI_SSID, ev->reason, (unsigned long)(wait / 1000));
        }
        esp_timer_stop(s_retry);
        esp_timer_start_once(s_retry, (uint64_t)wait * 1000);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_up = true;
        s_fails = 0;
        ESP_LOGI(TAG, "joined \"%s\" as %s  ->  http://%s/  or  http://%s.local/",
                 CONFIG_JKB_WIFI_SSID, s_ip, s_ip, CONFIG_JKB_MDNS_HOSTNAME);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "a device joined the access point");
    }
}

void wifi_start(void)
{
    s_sta = CONFIG_JKB_WIFI_SSID[0] != '\0';
    s_ap = strlen(CONFIG_JKB_AP_PASS) >= 8;
    if (CONFIG_JKB_AP_PASS[0] && !s_ap) {
        ESP_LOGE(TAG, "AP password is shorter than 8 characters - access point OFF");
    }
    if (!s_sta && !s_ap) {
        ESP_LOGE(TAG, "no Wi-Fi network and no access point set: the web page cannot be reached");
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (s_sta) {
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        /* The name the router shows in its client list. Without this it is
         * "espressif", the same as every other ESP on the network. */
        esp_netif_set_hostname(sta_netif, CONFIG_JKB_MDNS_HOSTNAME);
    }
    if (s_ap) {
        esp_netif_create_default_wifi_ap();
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &on_event, NULL, NULL));

    const esp_timer_create_args_t targs = {.callback = retry_cb, .name = "wifi_retry"};
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_retry));

    ESP_ERROR_CHECK(esp_wifi_set_mode(s_sta && s_ap ? WIFI_MODE_APSTA
                                      : s_sta ? WIFI_MODE_STA : WIFI_MODE_AP));

    if (s_sta) {
        wifi_config_t sta = {0};
        strncpy((char *)sta.sta.ssid, CONFIG_JKB_WIFI_SSID, sizeof(sta.sta.ssid) - 1);
        strncpy((char *)sta.sta.password, CONFIG_JKB_WIFI_PASS, sizeof(sta.sta.password) - 1);
        sta.sta.threshold.authmode = CONFIG_JKB_WIFI_PASS[0] ? WIFI_AUTH_WPA2_PSK
                                                             : WIFI_AUTH_OPEN;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    }
    if (s_ap) {
        wifi_config_t apc = {0};
        strncpy((char *)apc.ap.ssid, CONFIG_JKB_AP_SSID, sizeof(apc.ap.ssid) - 1);
        apc.ap.ssid_len = strlen(CONFIG_JKB_AP_SSID);
        strncpy((char *)apc.ap.password, CONFIG_JKB_AP_PASS, sizeof(apc.ap.password) - 1);
        apc.ap.authmode = WIFI_AUTH_WPA2_PSK;
        apc.ap.max_connection = 4;
        /* Once the station joins, the AP moves to the router's channel. */
        apc.ap.channel = 1;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apc));
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    /* No power save: mains powered, and it serves HTTP. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (s_ap) {
        ESP_LOGI(TAG, "access point \"%s\" up  ->  http://192.168.4.1/", CONFIG_JKB_AP_SSID);
    }

    /* Not fatal: the IP address still works. */
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(CONFIG_JKB_MDNS_HOSTNAME);
        mdns_instance_name_set("JK BMS CAN");
        mdns_service_add(NULL, "_http", "_tcp", CONFIG_JKB_HTTP_PORT, NULL, 0);
    } else {
        ESP_LOGW(TAG, "mdns failed; use the IP address");
    }
}

bool wifi_sta_up(void)
{
    return s_up;
}

const char *wifi_sta_ip(void)
{
    return s_ip;
}

int wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (s_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

bool wifi_ap_on(void)
{
    return s_ap;
}
