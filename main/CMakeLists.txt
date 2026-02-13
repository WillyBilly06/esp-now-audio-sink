idf_component_register(
    SRCS "main_adpcm.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_timer driver nvs_flash
)
