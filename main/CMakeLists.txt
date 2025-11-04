idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver log freertos esp_timer nvs_flash)