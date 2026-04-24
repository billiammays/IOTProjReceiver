idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_rmt esp_timer
                                  nvs_flash esp_event json)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
