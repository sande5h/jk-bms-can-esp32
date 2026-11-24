idf_component_register(
    SRCS
        "main.c"
        "can.c"
        "jkbms.c"
        "wifi.c"
        "webserver.c"
        "ota.c"
    INCLUDE_DIRS "."
    # The web page is a normal HTML file, built into the firmware.
    EMBED_TXTFILES "page.html"
    REQUIRES
        esp_driver_gpio
        esp_driver_twai
        esp_wifi
        esp_netif
        lwip
        esp_event
        esp_http_server
        esp_timer
        nvs_flash
        app_update
        esp_app_format
        bootloader_support
)
