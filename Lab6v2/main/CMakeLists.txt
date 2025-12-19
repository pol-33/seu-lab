idf_component_register(SRCS "esp_ibeacon_api.c"
                            "detector_nom_i_posicio.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart
                    INCLUDE_DIRS ".")
