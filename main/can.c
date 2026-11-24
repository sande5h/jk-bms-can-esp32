#include "can.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "can";

#define PIN_TX      CONFIG_JKB_CAN_TX_GPIO
#define PIN_RX      CONFIG_JKB_CAN_RX_GPIO
#define RX_QUEUE    64
#define LOG_LINES   40
#define LINE_LEN    64
#define MAX_IDS     64

#define NVS_NS      "can"
#define NVS_KEY     "kbps"

typedef struct {
    uint32_t id;
    bool     ext;
    uint8_t  len;
    uint8_t  data[8];
    uint32_t count;
    uint32_t last_ms;
    uint32_t period_ms;     /* smoothed gap; valid from the 2nd frame */
} id_entry_t;

static can_rx_handler_t s_on_rx;
static can_namer_t s_namer;

/* s_lock guards the id table, the log and the counters below it.
 * s_node_lock guards the node itself: the web page can rebuild it (bit rate)
 * while the CAN task reads its status. */
static SemaphoreHandle_t s_lock, s_node_lock;
static QueueHandle_t s_queue;
static twai_node_handle_t s_node;
static uint32_t s_kbps;

static id_entry_t s_ids[MAX_IDS];
static int s_nids;
static char s_log[LOG_LINES][LINE_LEN];
static int s_log_head, s_log_count;
static can_diag_t s_diag = {.state = CAN_STOPPED};

/* ── ISR side ─────────────────────────────────────────────────────── */

/* The driver fills this inside the ISR; it must outlive the callback. */
static uint8_t s_isr_buf[TWAI_FRAME_MAX_LEN];
static twai_frame_t s_isr_frame;
static volatile uint32_t s_n_bit, s_n_ack, s_n_stuff, s_n_form;

static IRAM_ATTR bool on_err(twai_node_handle_t node,
                             const twai_error_event_data_t *edata, void *ctx)
{
    (void)node;
    (void)ctx;
    if (edata->err_flags.bit_err)   s_n_bit++;
    if (edata->err_flags.ack_err)   s_n_ack++;
    if (edata->err_flags.stuff_err) s_n_stuff++;
    if (edata->err_flags.form_err)  s_n_form++;
    return false;
}

/* Copy out and post. No formatting, no logging, no blocking. */
static IRAM_ATTR bool on_rx_isr(twai_node_handle_t node,
                                const twai_rx_done_event_data_t *edata, void *ctx)
{
    (void)edata;
    (void)ctx;
    if (twai_node_receive_from_isr(node, &s_isr_frame) != ESP_OK) {
        return false;
    }
    can_frame_t f = {
        .ms  = (uint32_t)(esp_timer_get_time() / 1000),
        .id  = s_isr_frame.header.id,
        .len = s_isr_frame.header.dlc > 8 ? 8 : s_isr_frame.header.dlc,
        .ext = s_isr_frame.header.ide,
        .rtr = s_isr_frame.header.rtr,
    };
    memcpy(f.data, s_isr_buf, f.len);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_queue, &f, &woken);     /* full queue: drop, it is fine */
    return woken == pdTRUE;
}

/* RX pin sampler, 1 kHz. Reads the pad itself, so it works while TWAI owns
 * the pin. Too slow to see single bits, but it tells a quiet bus (~0 low)
 * from a busy one (a few %) from a stuck or unpowered one (1000). */
static volatile uint32_t s_pin_samples, s_pin_low;

static void pin_sample(void *arg)
{
    (void)arg;
    s_pin_samples++;
    if (gpio_get_level(PIN_RX) == 0) {
        s_pin_low++;
    }
}

/* ── log and id table ─────────────────────────────────────────────── */

static int format_frame(const can_frame_t *f, char *out, size_t out_len)
{
    /* Extended ids print as 8 digits, standard as 3, like candump. */
    int n = snprintf(out, out_len, f->ext ? "%08lX" : "%03lX", (unsigned long)f->id);
    if (f->rtr) {
        return n + snprintf(out + n, out_len - n, " rtr%u", f->len);
    }
    n += snprintf(out + n, out_len - n, " [%u]", f->len);
    for (int i = 0; i < f->len && n < (int)out_len - 4; i++) {
        n += snprintf(out + n, out_len - n, " %02X", f->data[i]);
    }
    return n;
}

/* Caller holds s_lock. */
static void log_line_locked(const can_frame_t *f, bool tx)
{
    char line[LINE_LEN];
    int n = snprintf(line, sizeof(line), "%6lu.%03lu %s ",
                     (unsigned long)(f->ms / 1000), (unsigned long)(f->ms % 1000),
                     tx ? "TX" : "  ");
    format_frame(f, line + n, sizeof(line) - n);
    snprintf(s_log[s_log_head], LINE_LEN, "%s", line);
    s_log_head = (s_log_head + 1) % LOG_LINES;
    if (s_log_count < LOG_LINES) {
        s_log_count++;
    }
}

/* Caller holds s_lock. Kept sorted (standard ids first), so the page never
 * sorts. 64 entries and ~30 frames a second: a linear walk is nothing. */
static void id_track_locked(const can_frame_t *f)
{
    int i = 0;
    while (i < s_nids && (s_ids[i].ext < f->ext ||
                          (s_ids[i].ext == f->ext && s_ids[i].id < f->id))) {
        i++;
    }
    id_entry_t *e;
    if (i < s_nids && s_ids[i].ext == f->ext && s_ids[i].id == f->id) {
        e = &s_ids[i];
        uint32_t gap = f->ms - e->last_ms;
        e->period_ms = e->count < 2 ? gap : (e->period_ms * 7 + gap) / 8;
    } else {
        if (s_nids == MAX_IDS) {
            return;
        }
        memmove(&s_ids[i + 1], &s_ids[i], (s_nids - i) * sizeof(s_ids[0]));
        s_nids++;
        e = &s_ids[i];
        memset(e, 0, sizeof(*e));
        e->id = f->id;
        e->ext = f->ext;
    }
    e->count++;
    e->last_ms = f->ms;
    e->len = f->len;
    memcpy(e->data, f->data, f->len);
}

size_t can_ids_text(char *out, size_t len)
{
    size_t n = snprintf(out, len, "ID        PERIOD    COUNT  DATA                     NAME\n");
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_nids == 0) {
        n += snprintf(out + n, len - n, "(no frames yet)\n");
    }
    for (int i = 0; i < s_nids && n < len - 100; i++) {
        const id_entry_t *e = &s_ids[i];
        char hex[28] = "";
        int h = 0;
        for (int k = 0; k < e->len; k++) {
            h += snprintf(hex + h, sizeof(hex) - h, "%s%02X", k ? " " : "", e->data[k]);
        }
        char per[12];
        if (e->count < 2) {
            snprintf(per, sizeof(per), "-");
        } else {
            snprintf(per, sizeof(per), "%lums", (unsigned long)e->period_ms);
        }
        const char *name = s_namer ? s_namer(e->id, e->ext) : NULL;
        bool stale = now - e->last_ms > 5000;
        n += snprintf(out + n, len - n, e->ext ? "%08lX  %-8s %6lu  %-24s %s%s\n"
                                               : "%03lX       %-8s %6lu  %-24s %s%s\n",
                      (unsigned long)e->id, per, (unsigned long)e->count, hex,
                      name ? name : "?", stale ? "  (stale)" : "");
    }
    xSemaphoreGive(s_lock);
    return n;
}

size_t can_log_text(char *out, size_t len)
{
    size_t n = 0;
    out[0] = '\0';
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Newest first. */
    for (int k = 0; k < s_log_count && n < len - LINE_LEN - 2; k++) {
        int idx = (s_log_head - 1 - k + LOG_LINES) % LOG_LINES;
        n += snprintf(out + n, len - n, "%s\n", s_log[idx]);
    }
    xSemaphoreGive(s_lock);
    return n;
}

void can_ids_clear(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_nids = 0;
    s_log_count = 0;
    s_log_head = 0;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "id table cleared");
}

/* ── node ─────────────────────────────────────────────────────────── */

/* Caller holds s_node_lock (or is can_start, before anything else runs). */
static esp_err_t start_node(uint32_t kbps)
{
    twai_onchip_node_config_t cfg = {
        .io_cfg.tx = PIN_TX,
        .io_cfg.rx = PIN_RX,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate = kbps * 1000,
        .tx_queue_depth = 4,
        .flags.enable_listen_only = false,
    };
    /* Idle bus is recessive = high. A pin left floating (no transceiver, or
     * one with no power) would read as noise and flood the ISR. A powered
     * transceiver drives the pin and this pull-up does not matter. */
    gpio_set_pull_mode(PIN_RX, GPIO_PULLUP_ONLY);

    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&cfg, &s_node), TAG, "new node");

    s_isr_frame.buffer = s_isr_buf;
    s_isr_frame.buffer_len = sizeof(s_isr_buf);

    twai_event_callbacks_t cbs = {.on_rx_done = on_rx_isr, .on_error = on_err};
    ESP_RETURN_ON_ERROR(twai_node_register_event_callbacks(s_node, &cbs, NULL),
                        TAG, "callbacks");
    ESP_RETURN_ON_ERROR(twai_node_enable(s_node), TAG, "enable");

    s_kbps = kbps;
    s_diag.state = CAN_ERR_ACTIVE;
    return ESP_OK;
}

static void stop_node(void)
{
    if (!s_node) {
        return;
    }
    twai_node_disable(s_node);      /* delete refuses unless disabled first */
    twai_node_delete(s_node);
    s_node = NULL;
    s_diag.state = CAN_STOPPED;
}

static void can_task(void *arg)
{
    (void)arg;
    int64_t next_sec = 0, next_beat = 0;
    uint32_t last_rx = 0, last_samples = 0, last_low = 0;

    while (1) {
        can_frame_t f;
        if (xQueueReceive(s_queue, &f, pdMS_TO_TICKS(100)) == pdTRUE) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_diag.rx++;
            log_line_locked(&f, false);
            id_track_locked(&f);
            xSemaphoreGive(s_lock);
            if (s_on_rx) {
                s_on_rx(&f);
            }
        }

        int64_t now = esp_timer_get_time();
        if (now < next_sec) {
            continue;
        }
        next_sec = now + 1000000;

        uint32_t ns = s_pin_samples - last_samples;
        uint32_t nl = s_pin_low - last_low;
        last_samples += ns;
        last_low += nl;

        twai_node_status_t st = {0};
        twai_node_record_t rec = {0};
        bool have = false;
        xSemaphoreTake(s_node_lock, portMAX_DELAY);
        if (s_node && twai_node_get_info(s_node, &st, &rec) == ESP_OK) {
            have = true;
            /* Bus-off recovery is not automatic. Without this one wiring
             * fault would stop the board until the next reboot. */
            if (st.state == TWAI_ERROR_BUS_OFF) {
                ESP_LOGW(TAG, "bus off - recovering");
                twai_node_recover(s_node);
                s_diag.bus_off_recoveries++;
            }
        }
        xSemaphoreGive(s_node_lock);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_diag.rate = s_diag.rx - last_rx;
        last_rx = s_diag.rx;
        s_diag.rxpin_low_pm = ns ? (uint16_t)((uint64_t)nl * 1000 / ns) : 0;
        if (have) {
            s_diag.state = (can_state_t)st.state;
            s_diag.tec = st.tx_error_count;
            s_diag.rec = st.rx_error_count;
        }
        s_diag.err_bit = s_n_bit;
        s_diag.err_ack = s_n_ack;
        s_diag.err_stuff = s_n_stuff;
        s_diag.err_form = s_n_form;
        can_diag_t d = s_diag;
        xSemaphoreGive(s_lock);

        /* Every 10 s, changed or not. "No frames" alone cannot tell a quiet
         * bus from one this board cannot read; the numbers beside it can. */
        if (now >= next_beat) {
            next_beat = now + 10000000;
            ESP_LOGI(TAG, "%lu kbit/s %s  %lu/s  rx %lu  tx %lu/%lu fail  tec %lu rec %lu"
                          "  rxpin low %u/1000  err bit %lu ack %lu stuff %lu form %lu",
                     (unsigned long)d.kbps, can_state_str(d.state),
                     (unsigned long)d.rate, (unsigned long)d.rx,
                     (unsigned long)d.tx_ok, (unsigned long)d.tx_fail,
                     (unsigned long)d.tec, (unsigned long)d.rec, d.rxpin_low_pm,
                     (unsigned long)d.err_bit, (unsigned long)d.err_ack,
                     (unsigned long)d.err_stuff, (unsigned long)d.err_form);
        }
    }
}

esp_err_t can_start(can_rx_handler_t on_rx, can_namer_t namer)
{
    s_on_rx = on_rx;
    s_namer = namer;
    s_lock = xSemaphoreCreateMutex();
    s_node_lock = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(RX_QUEUE, sizeof(can_frame_t));
    if (!s_lock || !s_node_lock || !s_queue) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t kbps = CONFIG_JKB_CAN_BITRATE_KBPS;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        uint32_t saved = 0;
        if (nvs_get_u32(nvs, NVS_KEY, &saved) == ESP_OK && saved) {
            kbps = saved;
        }
        nvs_close(nvs);
    }

    ESP_RETURN_ON_ERROR(start_node(kbps), TAG, "start");
    s_diag.kbps = kbps;
    xTaskCreate(can_task, "can", 4096, NULL, 5, NULL);

    const esp_timer_create_args_t pin_args = {.callback = pin_sample, .name = "canpin"};
    esp_timer_handle_t pin_timer;
    if (esp_timer_create(&pin_args, &pin_timer) == ESP_OK) {
        esp_timer_start_periodic(pin_timer, 1000);
    }
    ESP_LOGI(TAG, "TWAI up: TX=GPIO%d RX=GPIO%d @ %lu kbit/s, normal mode (ACK on)",
             PIN_TX, PIN_RX, (unsigned long)kbps);
    return ESP_OK;
}

esp_err_t can_set_kbps(uint32_t kbps)
{
    if (kbps != 125 && kbps != 250 && kbps != 500 && kbps != 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (kbps == s_kbps) {
        return ESP_OK;
    }
    xSemaphoreTake(s_node_lock, portMAX_DELAY);
    stop_node();
    esp_err_t err = start_node(kbps);
    xSemaphoreGive(s_node_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "restart at %lu kbit/s: %s", (unsigned long)kbps, esp_err_to_name(err));
        return err;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_diag.kbps = kbps;
    xSemaphoreGive(s_lock);

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, NVS_KEY, kbps);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "bit rate now %lu kbit/s", (unsigned long)kbps);
    return ESP_OK;
}

uint32_t can_kbps(void)
{
    return s_kbps;
}

esp_err_t can_send(uint32_t id, bool ext, const uint8_t *data, uint8_t len)
{
    if (len > 8 || (!ext && id > 0x7FF) || id > 0x1FFFFFFF) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[TWAI_FRAME_MAX_LEN] = {0};
    if (data && len) {
        memcpy(buf, data, len);
    }
    twai_frame_t frame = {
        .header.id = id,
        .header.dlc = len,
        .header.ide = ext,
        .buffer = buf,
        .buffer_len = len,
    };

    xSemaphoreTake(s_node_lock, portMAX_DELAY);
    esp_err_t err = s_node ? twai_node_transmit(s_node, &frame, 100) : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(s_node_lock);

    can_frame_t f = {.ms = (uint32_t)(esp_timer_get_time() / 1000),
                     .id = id, .len = len, .ext = ext};
    memcpy(f.data, buf, len);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (err == ESP_OK) {
        s_diag.tx_ok++;
        log_line_locked(&f, true);
    } else {
        s_diag.tx_fail++;
    }
    xSemaphoreGive(s_lock);

    char line[LINE_LEN];
    format_frame(&f, line, sizeof(line));
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "TX %s", line);
    } else {
        ESP_LOGW(TAG, "TX %s failed: %s", line, esp_err_to_name(err));
    }
    return err;
}

void can_get_diag(can_diag_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_diag;
    xSemaphoreGive(s_lock);
}

const char *can_state_str(can_state_t s)
{
    switch (s) {
    case CAN_ERR_ACTIVE:  return "ok";
    case CAN_ERR_WARNING: return "warning";
    case CAN_ERR_PASSIVE: return "PASSIVE";
    case CAN_ERR_BUS_OFF: return "BUS OFF";
    default:              return "stopped";
    }
}
