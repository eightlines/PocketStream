idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_eth esp_netif esp_event nvs_flash esp_lcd esp_h264 esp_driver_ppa esp_psram
                       INCLUDE_DIRS "")
