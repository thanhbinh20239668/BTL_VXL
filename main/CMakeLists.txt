idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ds3231 bme280 esp-pms u8g2 driver fatfs nvs_flash esp_wifi bt)