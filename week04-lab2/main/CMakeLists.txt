idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc esp_driver_gpio driver)

component_compile_options(-lm)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)