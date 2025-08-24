# idf_component_register(SRCS "smartconfig_main.c"
#                     INCLUDE_DIRS "."
#                     REQUIRES DS3231 )
idf_component_register(
    SRCS "smartconfig_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        DS3231
        NTP
        NVS
        driver          # I2C 驱动在这里
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        wpa_supplicant
)
