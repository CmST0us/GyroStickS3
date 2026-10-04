idf_component_register(
    SRCS "main.c" "board.c" "pm1.c" "imu.c" "recorder.c" "logstore.c" "lcd.c" "lcd_gfx.c" "ui.c" "buttons.c" "hostlink.c" "power.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_driver_ledc esp_driver_usb_serial_jtag esp_driver_uart esp_lcd
             esp_timer esp_partition esp_hw_support esp_system freertos log heap
             gyl imu_fifo bmi270_api
)
