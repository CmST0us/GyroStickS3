/* Debounce + click / long press state machine, free of hardware access (unit tested on the host). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BTN_DEBOUNCE_US 25000

typedef enum {
    BTN_EV_NONE = 0,
    BTN_EV_CLICK,       /* released before the long press time */
    BTN_EV_LONG,        /* held for the long press time (fires while still held) */
} btn_event_t;

typedef struct {
    bool    stable;      /* debounced state: true = pressed */
    bool    raw_last;
    int64_t raw_since;   /* when the raw level last changed */
    int64_t pressed_at;
    bool    long_fired;
    bool    swallow;     /* ignore the current press (it already did its job, e.g. woke the device) */
} btn_state_t;

static inline btn_event_t btn_step(btn_state_t *b, bool raw_pressed, int64_t now_us, int64_t long_us)
{
    if (raw_pressed != b->raw_last) {
        b->raw_last = raw_pressed;
        b->raw_since = now_us;
    }
    btn_event_t ev = BTN_EV_NONE;
    if (raw_pressed != b->stable && now_us - b->raw_since >= BTN_DEBOUNCE_US) {
        b->stable = raw_pressed;
        if (raw_pressed) {
            b->pressed_at = now_us;
            b->long_fired = false;
        } else if (b->swallow) {
            b->swallow = false;
        } else if (!b->long_fired) {
            ev = BTN_EV_CLICK;
        }
    }
    if (b->stable && !b->long_fired && !b->swallow && now_us - b->pressed_at >= long_us) {
        b->long_fired = true;
        ev = BTN_EV_LONG;
    }
    return ev;
}

/* Treat a press that is already in progress as consumed. */
static inline void btn_swallow(btn_state_t *b, bool pressed_now)
{
    b->swallow = pressed_now;
    b->stable = pressed_now;
    b->raw_last = pressed_now;
    b->long_fired = true;
}

/* While the screen is off, the first click of either button only wakes it up: the click is consumed so it
 * cannot start / stop a recording or open a menu by accident. Long presses are left alone (deliberate gesture).
 * Returns true when the screen has to be woken. */
static inline bool btn_wake_gate(bool screen_on, btn_event_t *main_ev, btn_event_t *aux_ev)
{
    if (screen_on || (*main_ev != BTN_EV_CLICK && *aux_ev != BTN_EV_CLICK)) {
        return false;
    }
    if (*main_ev == BTN_EV_CLICK) *main_ev = BTN_EV_NONE;
    if (*aux_ev == BTN_EV_CLICK) *aux_ev = BTN_EV_NONE;
    return true;
}

static inline bool btn_busy(const btn_state_t *b, int64_t now_us)
{
    return b->stable || b->raw_last || now_us - b->raw_since < 2 * BTN_DEBOUNCE_US;
}
