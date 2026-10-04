/* Debounced click / long press detection for the two StickS3 buttons (active low GPIOs). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "btn_logic.h"

typedef enum {
    BTN_MAIN = 0,
    BTN_AUX,
    BTN_COUNT,
} btn_id_t;

void        buttons_init(void);
/* Call at least every ~10 ms while any button is down; returns at most one event per button per call. */
btn_event_t buttons_poll(btn_id_t id, int64_t now_us);
/* Raw (undebounced) level: true when pressed. */
bool        buttons_pressed(btn_id_t id);
/* True while any button is pressed or still settling: the caller must keep polling quickly. */
bool        buttons_busy(int64_t now_us);
/* Ignore everything from this button until it has been released (used after a long press that woke the device). */
void        buttons_swallow_until_release(btn_id_t id);
/* Wait (bounded) until the button is released and stable. */
void        buttons_wait_release(btn_id_t id, int timeout_ms);
/* Enable wake-from-light-sleep on the buttons' low level. */
void        buttons_arm_light_sleep_wake(void);
/* GPIO number of a button. */
int         buttons_gpio(btn_id_t id);
