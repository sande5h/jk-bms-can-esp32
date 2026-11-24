/*
 * JK BMS over CAN — protocol "BMS-CAN V2.1" (docs/JK_BMS_CAN_V2.1.pdf).
 *
 * 250 kbit/s. The BMS address is 0xF4 (+ the device address set in the JK
 * app). Little-endian, except the charge request 0x1806E5F4.
 *
 * Read: everything the BMS broadcasts (status, cells, temps, capacity,
 * alarms, faults, switch state, charge request).
 * Write: ONE frame only, Ctrl_INFO 0x18F0F428 — charge MOSFET, discharge
 * MOSFET and balancer. There is no restart and no settings write over CAN.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "can.h"
#include "esp_err.h"

#define JK_MAX_CELLS 28     /* frames 0x18E0..0x18E6, 4 cells each */
#define JK_MAX_TEMPS 5

/* Frame groups, for "last seen" ages. */
typedef enum {
    JK_G_ST1,       /* 0x2F4    voltage, current, SOC */
    JK_G_CELLV,     /* 0x4F4    max/min cell */
    JK_G_CELLT,     /* 0x5F4    max/min/avg temp */
    JK_G_ALM,       /* 0x7F4    alarms (only sent while an alarm is on) */
    JK_G_ST2,       /* 18F1     capacity, cycles */
    JK_G_TEMPS,     /* 18F2     all temperature sensors */
    JK_G_FAULT,     /* 18F3     fault bits */
    JK_G_INFO,      /* 18F4     runtime, heater current, SOH */
    JK_G_SW,        /* 18F5     switch state */
    JK_G_CELLS,     /* 18E0..E6 cell voltages */
    JK_G_CHG,       /* 1806E5   charge request (only with a charger) */
    JK_G_COUNT
} jk_group_t;

typedef struct {
    uint32_t seen_ms[JK_G_COUNT];   /* 0 = never */
    uint32_t frames;                /* frames from this BMS */
    uint32_t unknown;               /* frames from this BMS we do not decode */

    /* 0x2F4 */
    uint16_t volt_dv;               /* 0.1 V */
    int16_t  curr_da;               /* 0.1 A, + = charging (V2.1) */
    uint8_t  soc;

    /* 0x4F4 — cell numbers are 1-based */
    uint16_t cell_max_mv, cell_min_mv;
    uint8_t  cell_max_no, cell_min_no;

    /* 0x5F4 — °C, already minus 50 */
    int16_t  t_max, t_min, t_avg;
    uint8_t  t_max_no, t_min_no;

    /* 0x7F4 — 2 bits per alarm, 0 none, 1 serious, 2 important, 3 general */
    uint32_t alarms;

    /* 18F1 */
    uint16_t remain_dah, full_dah, cycle_dah;   /* 0.1 Ah */
    uint16_t cycles;

    /* 18F2 */
    uint8_t  temp_mask;             /* bit n = sensor n+1 fitted */
    int16_t  temps[JK_MAX_TEMPS];   /* °C; only valid where present */
    bool     temp_present[JK_MAX_TEMPS];

    /* 18F3 — bit n = fault n, see jk_fault_name() */
    uint32_t faults;

    /* 18F4 */
    uint32_t runtime_s;
    uint16_t heat_ma;
    uint8_t  soh;

    /* 18F5 */
    uint8_t  sw;                    /* raw byte, see JK_SW_* */

    /* 18E0..18E6 */
    uint16_t cell_mv[JK_MAX_CELLS];
    uint8_t  n_cells;               /* highest non-zero cell seen */

    /* 1806E5F4, big-endian */
    uint16_t chg_req_dv, chg_req_da;
    uint8_t  chg_req_off;           /* 0 = charger on, 1 = off */
    uint8_t  chg_req_heat;          /* 0 = charge mode, 1 = heat mode */
} jk_state_t;

#define JK_SW_CHG     0x01
#define JK_SW_DSG     0x02
#define JK_SW_BAL     0x04          /* balancing right now */
#define JK_SW_HEAT    0x08
#define JK_SW_PLUG    0x10          /* charger plugged in */
#define JK_SW_ACC     0x20

#define JK_N_ALARMS   15
#define JK_N_FAULTS   18

void jk_init(void);

/* Feed every received CAN frame. Ignores frames from other devices. */
void jk_feed(const can_frame_t *f);

/* Copy of the current state. */
void jk_get(jk_state_t *out);

/* True if any frame from the BMS arrived in the last 3 s. */
bool jk_online(void);

/* Short name for an id, for the CAN id table. NULL if not a JK frame. */
const char *jk_id_name(uint32_t id, bool ext);

const char *jk_alarm_name(int n);   /* n = 0..14 */
const char *jk_fault_name(int n);   /* n = 0..17 */

/* Sends Ctrl_INFO. Each value: -1 = leave alone, 0 = off, 1 = on.
 * Only the switches that are not -1 are put in the mask, so the others are
 * not touched. The result is checked against the next switch-state frame;
 * jk_ctrl_status() says how that went. */
esp_err_t jk_control(int chg, int dsg, int bal);
const char *jk_ctrl_status(void);

/* Once a second from main: times out an unconfirmed control write. */
void jk_tick(void);

/* JSON object with everything, for the web page. */
size_t jk_json(char *out, size_t len);

/* One line for the serial log heartbeat. */
void jk_log_summary(void);
