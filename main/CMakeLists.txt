idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash esp_driver_gpio driver
                    INCLUDE_DIRS ""
                    REQUIRES micro_ros_espidf_component tb6612 as5600 diff_speed_ctrl pid)
