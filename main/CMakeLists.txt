idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_rd-03d
        esp_event
        esp_netif
        esp_wifi
        esp_driver_uart
        nvs_flash
)
