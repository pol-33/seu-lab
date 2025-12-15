idf_component_register(SRCS "blink_example_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt esp_timer driver)