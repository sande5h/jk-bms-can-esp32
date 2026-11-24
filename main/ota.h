/*
 * Firmware update over Wi-Fi.
 *
 * The web server streams an uploaded build/jk_bms_can.bin through here into
 * the OTA slot that is not running. Before anything is erased, the start of
 * the file is checked: it must be an ESP image, for this chip, built from
 * this project. A .bin from another project is refused, not flashed.
 * esp_ota_end() then checks the image's SHA-256 before the boot slot is
 * switched.
 *
 * The new firmware boots on trial. main.c calls ota_mark_good() after 30 s
 * of running; if the board resets before that, the bootloader boots the
 * old firmware again (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* One upload at a time. On any error, ota_error() says why in one short
 * sentence, and the upload is already aborted. */
esp_err_t ota_begin(size_t total);
esp_err_t ota_write(const uint8_t *data, size_t len);
esp_err_t ota_finish(void);       /* checks the image and sets the boot slot */
void ota_abort(void);
bool ota_active(void);
const char *ota_error(void);

/* The version of the image just written. */
const char *ota_new_version(void);

/* Reboots after a short delay, so the HTTP reply gets out first. */
void ota_restart_soon(void);

/* At boot: logs the running slot and version, and whether it is a new
 * image on trial or the last update was rolled back. */
void ota_boot_report(void);

const char *ota_running_version(void);

/* Confirms a new image on trial. Cheap to call often. */
void ota_mark_good(void);
