/*
 * JK BMS over CAN — reads the pack and switches its MOSFETs.
 *
 * Hardware: ESP32-S3 + CAN transceiver. TX = GPIO17, RX = GPIO18 (Kconfig).
 * Wiring, first flash and OTA are in README.md.
 */
#include "can.h"
#include "jkbms.h"
#include "ota.h"
#include "webserver.h"
#include "wifi.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "main";

#define MARK_GOOD_AFTER_S  30
#define HEARTBEAT_S        10

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ota_boot_report();

    jk_init();
    err = can_start(jk_feed, jk_id_name);
    if (err != ESP_OK) {
        /* Keep going: the web page still works, and OTA can fix the build. */
        ESP_LOGE(TAG, "CAN did not start: %s", esp_err_to_name(err));
    }

    wifi_start();
    webserver_start();

    for (int sec = 1;; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        jk_tick();
        if (sec == MARK_GOOD_AFTER_S) {
            ota_mark_good();
        }
        /* Every 10 s whatever happens, so a quiet log always means a dead
         * board, never "nothing changed". */
        if (sec % HEARTBEAT_S == 0) {
            jk_log_summary();
            ESP_LOGI(TAG, "wifi %s %s  heap %lu", wifi_sta_up() ? "up" : "down",
                     wifi_sta_ip(), (unsigned long)esp_get_free_heap_size());
        }
    }
}
