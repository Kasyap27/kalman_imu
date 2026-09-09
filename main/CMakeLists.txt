idf_component_register(SRCS "main.c" "icm20948.c" "attitude_ekf.c" "uart_debug.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2c esp_timer)
