idf_component_register(SRCS "blink_simple.c" "simple_lowpass.c"
                       PRIV_REQUIRES spi_flash driver esp_adc
                       INCLUDE_DIRS "")
