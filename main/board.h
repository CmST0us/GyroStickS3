/* M5Stack StickS3 (ESP32-S3-PICO-1-N8R8) pin map. */
#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"

/* internal I2C bus: M5PM1 power manager + BMI270 IMU */
#define PIN_I2C_SDA  GPIO_NUM_47
#define PIN_I2C_SCL  GPIO_NUM_48
#define I2C_ADDR_PM1 0x6E
#define I2C_ADDR_IMU 0x68

/* ST7789P3 135x240 LCD (powered through M5PM1 GPIO2 = "L3B" rail) */
#define PIN_LCD_MOSI GPIO_NUM_39
#define PIN_LCD_SCLK GPIO_NUM_40
#define PIN_LCD_DC   GPIO_NUM_45
#define PIN_LCD_CS   GPIO_NUM_41
#define PIN_LCD_RST  GPIO_NUM_21
#define PIN_LCD_BL   GPIO_NUM_38
#define LCD_OFFSET_X 52
#define LCD_OFFSET_Y 40

/* Create (once) and return the shared internal I2C bus. */
i2c_master_bus_handle_t board_i2c_bus(void);
