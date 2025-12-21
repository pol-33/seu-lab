idf_component_register(SRCS "master_v2.c" "esp_ibeacon_api.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart
                    INCLUDE_DIRS ".")
