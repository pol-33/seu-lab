idf_component_register(SRCS "esp_ibeacon_api.c"
                            "detector_posicio2.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS ".")
