#pragma once

#include <stdbool.h>

/* Joins CONFIG_JKB_WIFI_SSID (retrying for ever, with backoff) and, if
 * CONFIG_JKB_AP_PASS is set, also runs the board's own access point at
 * 192.168.4.1. Does not block. Call once, after nvs_flash_init(). */
void wifi_start(void);

bool wifi_sta_up(void);

/* "192.168.1.42", or "-" while not joined. */
const char *wifi_sta_ip(void);

/* Signal of the joined network in dBm, 0 if not joined. */
int wifi_rssi(void);

bool wifi_ap_on(void);
