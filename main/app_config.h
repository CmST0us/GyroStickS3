/* Compile time configuration derived from Kconfig. */
#pragma once

#include "sdkconfig.h"

#define FW_VERSION "GYROSTICK_0.1.0"

/* ---- how the stick sits on the camera ---- */
#if defined(CONFIG_GYROLOG_MOUNT_UP_USB_RIGHT)
#define GYROLOG_ORIENTATION "Zxy"
#define GYROLOG_UI_ROT 1
#elif defined(CONFIG_GYROLOG_MOUNT_BACK_USB_RIGHT)
#define GYROLOG_ORIENTATION "YxZ"
#define GYROLOG_UI_ROT 1
#elif defined(CONFIG_GYROLOG_MOUNT_UP_USB_LEFT)
#define GYROLOG_ORIENTATION "ZXY"
#define GYROLOG_UI_ROT 3
#elif defined(CONFIG_GYROLOG_MOUNT_BACK_USB_LEFT)
#define GYROLOG_ORIENTATION "yXZ"
#define GYROLOG_UI_ROT 3
#else
#define GYROLOG_ORIENTATION CONFIG_GYROLOG_ORIENTATION_CUSTOM
#if defined(CONFIG_GYROLOG_UI_ROT_CUSTOM_DOWN)
#define GYROLOG_UI_ROT 0
#elif defined(CONFIG_GYROLOG_UI_ROT_CUSTOM_UP)
#define GYROLOG_UI_ROT 2
#elif defined(CONFIG_GYROLOG_UI_ROT_CUSTOM_LEFT)
#define GYROLOG_UI_ROT 3
#else
#define GYROLOG_UI_ROT 1
#endif
#endif
/* UI rotation: 0 = USB port at the bottom (native portrait), 1 = at the right, 2 = top, 3 = left */

/* ---- IMU ---- */
#define GYROLOG_ODR_HZ      CONFIG_GYROLOG_ODR_HZ
#define GYROLOG_GYRO_DPS    CONFIG_GYROLOG_GYRO_DPS
#define GYROLOG_ACC_G       CONFIG_GYROLOG_ACC_G
#ifdef CONFIG_GYROLOG_LOG_ACCEL
#define GYROLOG_CHANNELS 6
#else
#define GYROLOG_CHANNELS 3
#endif

/* ---- buttons / timing ---- */
#define BTN_MAIN_GPIO      CONFIG_GYROLOG_MAIN_BUTTON_GPIO
#define BTN_AUX_GPIO       CONFIG_GYROLOG_AUX_BUTTON_GPIO
#define LONG_PRESS_MS      CONFIG_GYROLOG_LONG_PRESS_MS
#define IDLE_SLEEP_S       CONFIG_GYROLOG_IDLE_SLEEP_S
#define REC_SCREEN_S       CONFIG_GYROLOG_REC_SCREEN_S
#define LCD_BRIGHTNESS_PCT CONFIG_GYROLOG_LCD_BRIGHTNESS
#define FIFO_POLL_MS       CONFIG_GYROLOG_FIFO_POLL_MS
#define LOW_BATT_MV        CONFIG_GYROLOG_LOW_BATT_MV
#ifdef CONFIG_GYROLOG_LIGHT_SLEEP_REC
#define LIGHT_SLEEP_REC 1
#else
#define LIGHT_SLEEP_REC 0
#endif
