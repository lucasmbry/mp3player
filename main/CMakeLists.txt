idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        arduino-esp32
        adafruit_gfx
        adafruit_st7789
        adafruit_ili9341
        esp32encoder
        tjpg_decoder
        esp_timer
        esp_system
)

