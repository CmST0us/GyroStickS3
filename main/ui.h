/* On-device screens. The LCD is only powered while something is shown. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     recording;
    bool     usb;
    uint16_t batt_mv;
    int      batt_pct;
    uint32_t free_pages;
    uint32_t total_pages;
    uint32_t sessions;
    uint32_t rec_samples;
    uint32_t rec_seconds;
    uint32_t rec_bytes;
    uint32_t free_seconds;   /* estimated recording time that still fits, 0 = unknown */
    bool     imu_ok;
    bool     pm1_ok;
    bool     has_acc;
    float    acc_g[3];       /* BMI270 accelerometer, chip axes, only while idle */
} ui_info_t;

typedef enum {
    UI_NONE = 0,
    UI_READY,
    UI_REC,
    UI_SAVED,
    UI_STATUS,
    UI_MESSAGE,
    UI_ERASE_CONFIRM,
    UI_SLEEP,
} ui_screen_t;

/* Supplies fresh numbers for screens that update while visible (REC, STATUS). */
void ui_set_info_provider(void (*fn)(ui_info_t *out));

void ui_show(ui_screen_t s, int timeout_s);
void ui_message(const char *line1, const char *line2, uint16_t color, int timeout_s);
/* Turn the LCD off when the current screen timed out, refresh live screens. */
void ui_service(int64_t now_us);
void ui_off(void);
bool ui_visible(void);
ui_screen_t ui_current(void);
/* Time (us) the LCD should be switched off at, 0 if off. */
int64_t ui_off_deadline(void);
