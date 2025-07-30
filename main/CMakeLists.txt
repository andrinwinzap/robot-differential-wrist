idf_component_register(SRCS "main.c"
                            "home.c"
                            "esp32_serial_transport.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "../include"
                    REQUIRES micro_ros_espidf_component tb6612 as5600 diff_speed_ctrl pid nvs_flash )
