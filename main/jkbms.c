#include "jkbms.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "jk";

#define SA          (0xF4 + CONFIG_JKB_BMS_ADDR)   /* the BMS's source address */
#define CTRL_SA     0x28                           /* ours, as in the JK document */
#define ONLINE_MS   3000
#define ALARM_HOLD_MS 3000   /* 0x7F4 stops when alarms clear; no "all clear" frame */
#define CONFIRM_MS  3000

static SemaphoreHandle_t s_lock;
static jk_state_t s;

/* The control write waiting for a switch-state frame to confirm it. */
static struct {
    bool     active;
    uint8_t  mask;          /* JK_SW_CHG / JK_SW_DSG bits we asked to change */
    uint8_t  want;          /* the state we asked for, same bits */
    uint32_t sent_ms;
    char     what[32];
} s_pend;
static char s_ctrl_status[160] = "no control sent since boot";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/* ── names ────────────────────────────────────────────────────────── */

/* 0x7F4, 2 bits each. V2.1 marks 3, 4, 10, 12, 13 and 14 as reserved; the
 * names are from V1.02, which V2.1 says it stays compatible with. */
static const char *const k_alarms[JK_N_ALARMS] = {
    "cell over-voltage", "cell under-voltage", "pack over-voltage",
    "pack under-voltage", "cell voltage difference", "discharge over-current",
    "charge over-current", "over-temperature", "under-temperature",
    "temperature difference", "SOC too low", "insulation low",
    "HV interlock", "external comm fault", "internal comm fault",
};

/* 18F328F4, one bit each, in the order of the V2.1 table. The example in the
 * document does not match its own table for bytes 1 and 2; the table is used
 * here. The page also shows the raw bits, so a wrong name can be spotted. */
static const char *const k_faults[JK_N_FAULTS] = {
    "wire resistance", "MOSFET over-temp", "cell count mismatch",
    "current sensor fault", "cell over-voltage", "pack over-voltage",
    "charge over-current", "charge short circuit", "charge over-temp",
    "charge under-temp", "BMS internal comm", "cell under-voltage",
    "pack under-voltage", "discharge over-current", "discharge short circuit",
    "discharge over-temp", "charge MOSFET fault", "discharge MOSFET fault",
};

const char *jk_alarm_name(int n)
{
    return (n >= 0 && n < JK_N_ALARMS) ? k_alarms[n] : "?";
}

const char *jk_fault_name(int n)
{
    return (n >= 0 && n < JK_N_FAULTS) ? k_faults[n] : "?";
}

const char *jk_id_name(uint32_t id, bool ext)
{
    if (!ext) {
        if ((id & 0xFF) != SA) {
            return NULL;
        }
        switch (id >> 8) {
        case 0x2: return "JK status V/A/SOC";
        case 0x4: return "JK max/min cell";
        case 0x5: return "JK max/min temp";
        case 0x7: return "JK alarms";
        default:  return "JK ?";
        }
    }
    if (id == ((0x18F0u << 16) | (SA << 8) | CTRL_SA)) {
        return "JK control (to BMS)";
    }
    if ((id & 0xFF) != SA) {
        return NULL;
    }
    if (id == (0x1806E500u | SA)) {
        return "JK charge request";
    }
    if ((id & 0xFF00FF00u) != 0x18002800u) {
        return "JK ?";
    }
    uint8_t pf = (id >> 16) & 0xFF;
    switch (pf) {
    case 0xF1: return "JK capacity/cycles";
    case 0xF2: return "JK all temps";
    case 0xF3: return "JK faults";
    case 0xF4: return "JK runtime/SOH";
    case 0xF5: return "JK switch state";
    default:
        if (pf >= 0xE0 && pf <= 0xE6) {
            return "JK cell voltages";
        }
        return "JK ?";
    }
}

/* ── decode ───────────────────────────────────────────────────────── */

static const char *sw_name(uint8_t bit)
{
    return bit == JK_SW_CHG ? "CHG" : bit == JK_SW_DSG ? "DSG" : "?";
}

/* Caller holds s_lock. A new switch-state frame arrived: does it show what
 * the last control write asked for? */
static void check_pending_locked(uint32_t ms)
{
    if (!s_pend.active || (int32_t)(ms - s_pend.sent_ms) < 0) {
        return;
    }
    if ((s.sw & s_pend.mask) == s_pend.want) {
        snprintf(s_ctrl_status, sizeof(s_ctrl_status), "%s: CONFIRMED by the BMS in %lu ms",
                 s_pend.what, (unsigned long)(ms - s_pend.sent_ms));
        ESP_LOGW(TAG, "%s", s_ctrl_status);
        s_pend.active = false;
    }
}

/* Caller holds s_lock. Returns false if the frame is from this BMS but not
 * one we decode. */
static bool decode_std_locked(uint8_t func, const uint8_t *d, uint32_t ms)
{
    switch (func) {
    case 0x2:
        s.volt_dv = le16(d);
        /* 0.1 A with a -400 A offset: raw 4000 = 0 A. */
        s.curr_da = (int16_t)((int32_t)le16(d + 2) - 4000);
        s.soc = d[4];
        s.seen_ms[JK_G_ST1] = ms;
        return true;
    case 0x4:
        s.cell_max_mv = le16(d);
        s.cell_max_no = d[2];
        s.cell_min_mv = le16(d + 3);
        s.cell_min_no = d[5];
        s.seen_ms[JK_G_CELLV] = ms;
        return true;
    case 0x5:
        s.t_max = (int16_t)d[0] - 50;
        s.t_max_no = d[1];
        s.t_min = (int16_t)d[2] - 50;
        s.t_min_no = d[3];
        s.t_avg = (int16_t)d[4] - 50;
        s.seen_ms[JK_G_CELLT] = ms;
        return true;
    case 0x7:
        s.alarms = le32(d);
        s.seen_ms[JK_G_ALM] = ms;
        return true;
    default:
        return false;
    }
}

static bool decode_ext_locked(uint32_t id, const uint8_t *d, uint8_t len, uint32_t ms)
{
    if (id == (0x1806E500u | SA)) {
        /* The only big-endian frame in the protocol. */
        s.chg_req_dv = be16(d);
        s.chg_req_da = be16(d + 2);
        s.chg_req_off = d[4];
        s.chg_req_heat = d[5];
        s.seen_ms[JK_G_CHG] = ms;
        return true;
    }
    if ((id & 0xFF00FF00u) != 0x18002800u) {
        return false;
    }
    uint8_t pf = (id >> 16) & 0xFF;

    if (pf >= 0xE0 && pf <= 0xE6) {
        int base = (pf - 0xE0) * 4;
        for (int i = 0; i < 4 && base + i < JK_MAX_CELLS && 2 * i + 1 < len; i++) {
            uint16_t mv = le16(d + 2 * i);
            s.cell_mv[base + i] = mv;
            /* Unused slots in the last frame are 00 00. */
            if (mv && base + i + 1 > s.n_cells) {
                s.n_cells = base + i + 1;
            }
        }
        s.seen_ms[JK_G_CELLS] = ms;
        return true;
    }

    switch (pf) {
    case 0xF1:
        s.remain_dah = le16(d);
        s.full_dah = le16(d + 2);
        s.cycle_dah = le16(d + 4);
        s.cycles = le16(d + 6);
        s.seen_ms[JK_G_ST2] = ms;
        return true;
    case 0xF2:
        s.temp_mask = d[0];
        for (int i = 0; i < JK_MAX_TEMPS; i++) {
            /* FF = no sensor. */
            s.temp_present[i] = d[1 + i] != 0xFF;
            s.temps[i] = (int16_t)d[1 + i] - 50;
        }
        s.seen_ms[JK_G_TEMPS] = ms;
        return true;
    case 0xF3:
        s.faults = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)(d[2] & 0x03) << 16);
        s.seen_ms[JK_G_FAULT] = ms;
        return true;
    case 0xF4:
        s.runtime_s = le32(d);
        s.heat_ma = le16(d + 4);
        s.soh = d[6];
        s.seen_ms[JK_G_INFO] = ms;
        return true;
    case 0xF5:
        s.sw = d[0];
        s.seen_ms[JK_G_SW] = ms;
        check_pending_locked(ms);
        return true;
    default:
        return false;
    }
}

void jk_feed(const can_frame_t *f)
{
    if (f->rtr || (f->id & 0xFF) != SA) {
        return;
    }
    /* Pad short frames, so a decoder never reads past what arrived. */
    uint8_t d[8] = {0};
    memcpy(d, f->data, f->len);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Standard ids are 11 bits: the function code is the top 3. */
    bool known = f->ext ? decode_ext_locked(f->id, d, f->len, f->ms)
                        : decode_std_locked((uint8_t)(f->id >> 8), d, f->ms);
    s.frames++;
    if (!known) {
        s.unknown++;
    }
    xSemaphoreGive(s_lock);
}

void jk_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "JK BMS-CAN V2.1, BMS address 0x%02X, control id 0x%08lX",
             SA, (unsigned long)((0x18F0u << 16) | (SA << 8) | CTRL_SA));
}

void jk_get(jk_state_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s;
    xSemaphoreGive(s_lock);
}

static bool fresh(const jk_state_t *st, jk_group_t g, uint32_t now, uint32_t max_ms)
{
    return st->seen_ms[g] && now - st->seen_ms[g] <= max_ms;
}

bool jk_online(void)
{
    uint32_t now = now_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool on = false;
    for (int g = 0; g < JK_G_COUNT && !on; g++) {
        on = fresh(&s, g, now, ONLINE_MS);
    }
    xSemaphoreGive(s_lock);
    return on;
}

/* ── control ──────────────────────────────────────────────────────── */

esp_err_t jk_control(int chg, int dsg, int bal)
{
    uint8_t d[8] = {0};
    uint8_t mask = 0, want = 0;
    char what[32] = "";
    int n = 0;

    if (chg >= 0) {
        d[0] |= 0x01;
        d[1] = chg ? 1 : 0;
        mask |= JK_SW_CHG;
        want |= chg ? JK_SW_CHG : 0;
        n += snprintf(what + n, sizeof(what) - n, "CHG %s ", chg ? "on" : "off");
    }
    if (dsg >= 0) {
        d[0] |= 0x02;
        d[2] = dsg ? 1 : 0;
        mask |= JK_SW_DSG;
        want |= dsg ? JK_SW_DSG : 0;
        n += snprintf(what + n, sizeof(what) - n, "DSG %s ", dsg ? "on" : "off");
    }
    if (bal >= 0) {
        d[0] |= 0x04;
        d[3] = bal ? 1 : 0;
        n += snprintf(what + n, sizeof(what) - n, "BAL %s ", bal ? "on" : "off");
    }
    if (d[0] == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (n > 0) {
        what[n - 1] = '\0';     /* trailing space */
    }

    uint32_t id = (0x18F0u << 16) | (SA << 8) | CTRL_SA;
    ESP_LOGW(TAG, "control: %s", what);
    esp_err_t err = can_send(id, true, d, 8);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (err != ESP_OK) {
        snprintf(s_ctrl_status, sizeof(s_ctrl_status), "%s: send FAILED (%s) - no node ACKed it",
                 what, esp_err_to_name(err));
        s_pend.active = false;
    } else if (mask && fresh(&s, JK_G_SW, now_ms(), ONLINE_MS) && (s.sw & mask) == want) {
        /* Nothing would change, so the next frame proves nothing. */
        s_pend.active = false;
        snprintf(s_ctrl_status, sizeof(s_ctrl_status),
                 "%s: sent, but the BMS was already in that state - nothing to confirm", what);
    } else if (mask) {
        s_pend.active = true;
        s_pend.mask = mask;
        s_pend.want = want;
        s_pend.sent_ms = now_ms();
        snprintf(s_pend.what, sizeof(s_pend.what), "%s", what);
        snprintf(s_ctrl_status, sizeof(s_ctrl_status), "%s: sent, waiting for the switch-state frame",
                 what);
    } else {
        /* 18F5 bit 2 is "balancing right now", not "balancer enabled", so
         * no frame can confirm a balancer write. */
        s_pend.active = false;
        snprintf(s_ctrl_status, sizeof(s_ctrl_status),
                 "%s: sent. No frame reports the balancer setting - check it in the JK app", what);
    }
    xSemaphoreGive(s_lock);
    return err;
}

const char *jk_ctrl_status(void)
{
    return s_ctrl_status;
}

void jk_tick(void)
{
    uint32_t now = now_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_pend.active && now - s_pend.sent_ms > CONFIRM_MS) {
        uint8_t got = s.sw & s_pend.mask;
        char seen[24] = "";
        int n = 0;
        for (uint8_t bit = JK_SW_CHG; bit <= JK_SW_DSG; bit <<= 1) {
            if (s_pend.mask & bit) {
                n += snprintf(seen + n, sizeof(seen) - n, " %s %s", sw_name(bit),
                              (got & bit) ? "on" : "off");
            }
        }
        snprintf(s_ctrl_status, sizeof(s_ctrl_status),
                 "%s: NOT confirmed after %d s (BMS still says%s). It may ignore CAN control.",
                 s_pend.what, CONFIRM_MS / 1000, seen);
        ESP_LOGW(TAG, "%s", s_ctrl_status);
        s_pend.active = false;
    }
    /* Alarms are only sent while one is active. When the frame stops, they
     * have cleared. */
    if (s.alarms && !fresh(&s, JK_G_ALM, now, ALARM_HOLD_MS)) {
        s.alarms = 0;
    }
    xSemaphoreGive(s_lock);
}

/* ── output ───────────────────────────────────────────────────────── */

static void ap(char *out, size_t len, size_t *n, const char *fmt, ...)
{
    if (*n >= len) {
        return;
    }
    va_list va;
    va_start(va, fmt);
    int w = vsnprintf(out + *n, len - *n, fmt, va);
    va_end(va);
    if (w > 0) {
        *n += (size_t)w;
    }
}

static long age(const jk_state_t *st, jk_group_t g, uint32_t now)
{
    return st->seen_ms[g] ? (long)(now - st->seen_ms[g]) : -1;
}

/* Raw units only (0.1 V, 0.1 A, mV, °C). The page does the scaling. */
size_t jk_json(char *out, size_t len)
{
    jk_state_t st;
    jk_get(&st);
    uint32_t now = now_ms();
    size_t n = 0;

    static const char *const gnames[JK_G_COUNT] = {
        "st1", "cellv", "cellt", "alm", "st2", "temps", "fault", "info", "sw", "cells", "chg",
    };

    ap(out, len, &n, "{\"online\":%s,\"addr\":%d,\"frames\":%lu,\"unknown\":%lu,\"age\":{",
       jk_online() ? "true" : "false", CONFIG_JKB_BMS_ADDR,
       (unsigned long)st.frames, (unsigned long)st.unknown);
    for (int g = 0; g < JK_G_COUNT; g++) {
        ap(out, len, &n, "%s\"%s\":%ld", g ? "," : "", gnames[g], age(&st, g, now));
    }
    ap(out, len, &n, "},");

    ap(out, len, &n, "\"volt_dv\":%u,\"curr_da\":%d,\"soc\":%u,",
       st.volt_dv, st.curr_da, st.soc);
    ap(out, len, &n, "\"cmax\":%u,\"cmax_no\":%u,\"cmin\":%u,\"cmin_no\":%u,",
       st.cell_max_mv, st.cell_max_no, st.cell_min_mv, st.cell_min_no);
    ap(out, len, &n, "\"tmax\":%d,\"tmax_no\":%u,\"tmin\":%d,\"tmin_no\":%u,\"tavg\":%d,",
       st.t_max, st.t_max_no, st.t_min, st.t_min_no, st.t_avg);
    ap(out, len, &n, "\"remain_dah\":%u,\"full_dah\":%u,\"cycle_dah\":%u,\"cycles\":%u,",
       st.remain_dah, st.full_dah, st.cycle_dah, st.cycles);
    ap(out, len, &n, "\"runtime_s\":%lu,\"heat_ma\":%u,\"soh\":%u,\"sw\":%u,",
       (unsigned long)st.runtime_s, st.heat_ma, st.soh, st.sw);

    ap(out, len, &n, "\"temp_mask\":%u,\"temps\":[", st.temp_mask);
    for (int i = 0; i < JK_MAX_TEMPS; i++) {
        if (st.temp_present[i]) {
            ap(out, len, &n, "%s%d", i ? "," : "", st.temps[i]);
        } else {
            ap(out, len, &n, "%snull", i ? "," : "");
        }
    }
    ap(out, len, &n, "],\"cells\":[");
    for (int i = 0; i < st.n_cells; i++) {
        ap(out, len, &n, "%s%u", i ? "," : "", st.cell_mv[i]);
    }

    ap(out, len, &n, "],\"alarms_raw\":%lu,\"alarms\":[", (unsigned long)st.alarms);
    bool first = true;
    for (int i = 0; i < JK_N_ALARMS; i++) {
        int lvl = (st.alarms >> (2 * i)) & 3;
        if (lvl) {
            ap(out, len, &n, "%s{\"name\":\"%s\",\"level\":%d}", first ? "" : ",",
               k_alarms[i], lvl);
            first = false;
        }
    }
    ap(out, len, &n, "],\"faults_raw\":%lu,\"faults\":[", (unsigned long)st.faults);
    first = true;
    for (int i = 0; i < JK_N_FAULTS; i++) {
        if (st.faults & (1u << i)) {
            ap(out, len, &n, "%s\"%s\"", first ? "" : ",", k_faults[i]);
            first = false;
        }
    }
    ap(out, len, &n, "],\"chg_req\":{\"dv\":%u,\"da\":%u,\"off\":%u,\"heat\":%u},",
       st.chg_req_dv, st.chg_req_da, st.chg_req_off, st.chg_req_heat);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    ap(out, len, &n, "\"ctrl_status\":\"%s\",\"ctrl_pending\":%s}",
       s_ctrl_status, s_pend.active ? "true" : "false");
    xSemaphoreGive(s_lock);
    return n;
}

void jk_log_summary(void)
{
    jk_state_t st;
    jk_get(&st);
    uint32_t now = now_ms();

    if (!jk_online()) {
        ESP_LOGW(TAG, "BMS offline: no frame from address 0x%02X for %s  (frames since boot %lu)",
                 SA, st.frames ? "3+ s" : "ever", (unsigned long)st.frames);
        return;
    }
    int a = st.curr_da < 0 ? -st.curr_da : st.curr_da;
    ESP_LOGI(TAG, "BMS %u.%uV %s%d.%dA %u%%  cells %u..%u mV (d %d)  T %d..%d C"
                  "  CHG %d DSG %d BAL %d  alarms %08lX faults %05lX  frames %lu%s",
             st.volt_dv / 10, st.volt_dv % 10, st.curr_da < 0 ? "-" : "", a / 10, a % 10,
             st.soc, st.cell_min_mv, st.cell_max_mv, st.cell_max_mv - st.cell_min_mv,
             st.t_min, st.t_max,
             !!(st.sw & JK_SW_CHG), !!(st.sw & JK_SW_DSG), !!(st.sw & JK_SW_BAL),
             (unsigned long)st.alarms, (unsigned long)st.faults, (unsigned long)st.frames,
             fresh(&st, JK_G_SW, now, ONLINE_MS) ? "" : "  (no switch-state frame)");
}
