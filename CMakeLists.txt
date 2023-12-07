idf_component_register(SRCS "ads101x.c"
                    INCLUDE_DIRS "include"
                    REQUIRES i2c_bus esp_timer driver)
