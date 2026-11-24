/*
 * CAN on the on-chip TWAI controller.
 *
 * Normal mode, so this board ACKs every frame. With only the BMS and this
 * board on the bus, listen-only would mean nobody ACKs the BMS.
 * Nothing is sent unless a person asks for it (control buttons, raw send).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t ms;            /* uptime when it arrived */
    uint32_t id;
    uint8_t  len;
    bool     ext;           /* 29-bit id */
    bool     rtr;
    uint8_t  data[8];
} can_frame_t;

/* Called from the CAN task (not the ISR) for every frame received. */
typedef void (*can_rx_handler_t)(const can_frame_t *f);

/* Gives a short name for an id in the id table, or NULL. */
typedef const char *(*can_namer_t)(uint32_t id, bool ext);

typedef enum {
    CAN_ERR_ACTIVE = 0,     /* same order as twai_error_state_t */
    CAN_ERR_WARNING,
    CAN_ERR_PASSIVE,
    CAN_ERR_BUS_OFF,
    CAN_STOPPED,
} can_state_t;

typedef struct {
    uint32_t    kbps;
    can_state_t state;
    uint32_t    tec, rec;           /* error counters from the controller */
    uint32_t    rx;                 /* good frames since boot */
    uint32_t    rate;               /* good frames in the last second */
    uint32_t    tx_ok, tx_fail;
    uint32_t    err_bit, err_ack, err_stuff, err_form;
    uint32_t    bus_off_recoveries;
    /* How often the RX pin read LOW, per 1000 samples in the last second.
     * ~0 = idle bus. A few % = frames. 1000 = stuck dominant or no
     * transceiver power. See README. */
    uint16_t    rxpin_low_pm;
} can_diag_t;

/* Reads the saved bit rate and starts. Call once. */
esp_err_t can_start(can_rx_handler_t on_rx, can_namer_t namer);

/* 125, 250, 500 or 1000. Saved, so it survives a reboot. */
esp_err_t can_set_kbps(uint32_t kbps);
uint32_t can_kbps(void);

/* Blocks up to 100 ms for room in the TX queue. */
esp_err_t can_send(uint32_t id, bool ext, const uint8_t *data, uint8_t len);

void can_get_diag(can_diag_t *out);
const char *can_state_str(can_state_t s);

/* Plain-text id table and the newest frames, for the web page. */
size_t can_ids_text(char *out, size_t len);
size_t can_log_text(char *out, size_t len);
void can_ids_clear(void);
