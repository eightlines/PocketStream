idf_component_register(SRCS "my_jpeg_decoder.c" "yuv_rgb.c" "h264.c" "main.c" "ethernet.c" "display.c"
                       PRIV_REQUIRES spi_flash esp_eth esp_netif esp_event nvs_flash esp_lcd esp_h264 esp_driver_ppa esp_psram esp_timer unity lwip driver esp_jpeg esp_media_protocols media_lib_sal
                       INCLUDE_DIRS ".")
