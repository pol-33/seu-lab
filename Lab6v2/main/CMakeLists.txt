idf_component_register(SRCS "master.c" "esp_ibeacon_api.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart
                    INCLUDE_DIRS ".")
