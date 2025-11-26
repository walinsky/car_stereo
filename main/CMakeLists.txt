idf_component_register(
    SRCS 
        "main.c"
        "buttons.c"
        "car_stereo_state.c"
        "display.c"
    REQUIRES 
        bt 
        nvs_flash
    PRIV_REQUIRES 
        spi_flash
        esp_adc
        driver
        
    INCLUDE_DIRS "."
)
