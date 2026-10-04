#pragma once

#include <stdbool.h>

/* Call first thing after boot: undo the pin holds / RTC pad config of a previous deep sleep. */
void power_after_wake(void);
/* Wakeup was caused by the main button (deep sleep exit). */
bool power_woke_by_button(void);
/* Turn everything off and deep sleep; the main button (long press) wakes the device. Does not return. */
void power_deep_sleep(void) __attribute__((noreturn));
