/* Minimal driver for the M5PM1 power management chip (I2C 0x6E) on the StickS3. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t pm1_init(void);
bool      pm1_present(void);
/* LCD supply rail (PM1 GPIO2 / "L3B"). */
esp_err_t pm1_lcd_power(bool on);
/* Battery voltage in mV, 0 on error. */
uint16_t  pm1_battery_mv(void);
/* USB (VIN) voltage in mV, 0 on error / not present. */
uint16_t  pm1_vin_mv(void);
bool      pm1_usb_present(void);
/* Percent estimate (0..100) from a battery voltage. */
int       pm1_battery_percent(uint16_t mv);
