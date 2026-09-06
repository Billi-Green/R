# Radar Sensor Component CMakeLists.txt

idf_component_register(
    SRCS "esp_rd-03d.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_driver_gpio
        esp_driver_uart
        esp_common
        log
)
