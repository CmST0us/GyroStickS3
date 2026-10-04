#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gyl_format.h"
#include "lcd.h"

static void (*s_provider)(ui_info_t *);
static ui_screen_t s_cur = UI_NONE;
static int64_t s_off_at;
static int64_t s_next_refresh;
static char s_msg1[24], s_msg2[24];
static uint16_t s_msg_color;

void ui_set_info_provider(void (*fn)(ui_info_t *out))
{
    s_provider = fn;
}

static void fmt_duration(char *buf, size_t n, uint32_t sec)
{
    snprintf(buf, n, "%u:%02u:%02u", (unsigned)(sec / 3600), (unsigned)(sec / 60 % 60), (unsigned)(sec % 60));
}

static void fmt_free(char *buf, size_t n, const ui_info_t *i)
{
    uint32_t kb = i->free_pages * (GYL_PAYLOAD_MAX / 1024);
    if (kb >= 1024) {
        snprintf(buf, n, "FREE %u.%uMB", (unsigned)(kb / 1024), (unsigned)(kb % 1024 * 10 / 1024));
    } else {
        snprintf(buf, n, "FREE %uKB", (unsigned)kb);
    }
}

static void fmt_free_time(char *buf, size_t n, const ui_info_t *i)
{
    if (i->free_seconds) {
        uint32_t m = i->free_seconds / 60;
        snprintf(buf, n, "~%uh%02um LEFT", (unsigned)(m / 60), (unsigned)(m % 60));
    } else {
        snprintf(buf, n, "%s", "");
    }
}

static void draw_header(const ui_info_t *i, const char *title, uint16_t color)
{
    int w = lcd_width();
    lcd_fill(LCD_BLACK);
    lcd_rect(0, 0, w, 20, color);
    lcd_text(6, 2, title, 2, LCD_BLACK, color);
    char b[20];
    snprintf(b, sizeof(b), "%d%%%s", i->batt_pct, i->usb ? "+" : "");
    lcd_text(w - lcd_text_width(b, 2) - 6, 2, b, 2, LCD_BLACK, color);
}

static void draw(ui_screen_t s, const ui_info_t *i)
{
    char b[40];
    int w = lcd_width(), h = lcd_height();
    switch (s) {
    case UI_READY:
        draw_header(i, "GYRO LOG", LCD_GREEN);
        lcd_text(6, 30, "READY", 4, LCD_GREEN, LCD_BLACK);
        fmt_free(b, sizeof(b), i);
        lcd_text(6, 70, b, 2, LCD_WHITE, LCD_BLACK);
        fmt_free_time(b, sizeof(b), i);
        lcd_text(6, 90, b, 2, LCD_GREY, LCD_BLACK);
        lcd_text(6, h - 12, "CLICK=REC HOLD=SLEEP", 1, LCD_GREY, LCD_BLACK);
        break;
    case UI_REC:
        draw_header(i, "RECORDING", LCD_RED);
        fmt_duration(b, sizeof(b), i->rec_seconds);
        lcd_text(6, 30, b, 3, LCD_WHITE, LCD_BLACK);
        snprintf(b, sizeof(b), "%uHz %uKB", GYROLOG_ODR_HZ, (unsigned)(i->rec_bytes / 1024));
        lcd_text(6, 64, b, 2, LCD_GREY, LCD_BLACK);
        fmt_free(b, sizeof(b), i);
        lcd_text(6, 84, b, 2, LCD_WHITE, LCD_BLACK);
        fmt_free_time(b, sizeof(b), i);
        lcd_text(6, 104, b, 2, LCD_GREY, LCD_BLACK);
        break;
    case UI_SAVED:
        draw_header(i, "SAVED", LCD_BLUE);
        fmt_duration(b, sizeof(b), i->rec_seconds);
        lcd_text(6, 30, b, 3, LCD_WHITE, LCD_BLACK);
        snprintf(b, sizeof(b), "%uKB stored", (unsigned)(i->rec_bytes / 1024));
        lcd_text(6, 64, b, 2, LCD_GREY, LCD_BLACK);
        fmt_free(b, sizeof(b), i);
        lcd_text(6, 84, b, 2, LCD_WHITE, LCD_BLACK);
        fmt_free_time(b, sizeof(b), i);
        lcd_text(6, 104, b, 2, LCD_GREY, LCD_BLACK);
        break;
    case UI_STATUS:
        draw_header(i, "STATUS", LCD_YELLOW);
        snprintf(b, sizeof(b), "BAT %umV %d%%", i->batt_mv, i->batt_pct);
        lcd_text(6, 26, b, 2, LCD_WHITE, LCD_BLACK);
        snprintf(b, sizeof(b), "USB:%s IMU:%s", i->usb ? "ON" : "--", i->imu_ok ? "OK" : "ERR");
        lcd_text(6, 46, b, 2, i->imu_ok ? LCD_WHITE : LCD_RED, LCD_BLACK);
        fmt_free(b, sizeof(b), i);
        lcd_text(6, 66, b, 2, LCD_WHITE, LCD_BLACK);
        snprintf(b, sizeof(b), "%urec %uHz %s", (unsigned)i->sessions, GYROLOG_ODR_HZ, GYROLOG_ORIENTATION);
        lcd_text(6, 86, b, 2, LCD_GREY, LCD_BLACK);
        if (i->has_acc) {
            snprintf(b, sizeof(b), "%+.1f %+.1f %+.1f", (double)i->acc_g[0], (double)i->acc_g[1], (double)i->acc_g[2]);
        } else {
            snprintf(b, sizeof(b), "%s", "ACC --");
        }
        lcd_text(6, 106, b, 2, LCD_GREY, LCD_BLACK);
        lcd_text(6, h - 10, "HOLD AUX=ERASE", 1, LCD_GREY, LCD_BLACK);
        break;
    case UI_ERASE_CONFIRM:
        draw_header(i, "ERASE?", LCD_RED);
        lcd_text(6, 30, "WIPE ALL LOGS", 2, LCD_WHITE, LCD_BLACK);
        lcd_text(6, 56, "MAIN CLICK=YES", 2, LCD_RED, LCD_BLACK);
        lcd_text(6, 76, "AUX CLICK=NO", 2, LCD_GREEN, LCD_BLACK);
        break;
    case UI_MESSAGE:
        lcd_fill(LCD_BLACK);
        lcd_rect(0, 0, w, 6, s_msg_color);
        lcd_text(6, 30, s_msg1, 3, s_msg_color, LCD_BLACK);
        lcd_text(6, 70, s_msg2, 2, LCD_WHITE, LCD_BLACK);
        break;
    case UI_SLEEP:
        lcd_fill(LCD_BLACK);
        lcd_text(6, 50, "SLEEP", 4, LCD_GREY, LCD_BLACK);
        break;
    default:
        break;
    }
    lcd_flush();
}

static void collect(ui_info_t *i)
{
    memset(i, 0, sizeof(*i));
    if (s_provider) {
        s_provider(i);
    }
}

void ui_show(ui_screen_t s, int timeout_s)
{
#ifdef CONFIG_GYROLOG_SELFTEST
    ESP_LOGI("ui", "screen %d (timeout %d s)", (int)s, timeout_s); /* no LCD under QEMU */
    return;
#endif
    if (lcd_power_on() != ESP_OK) {
        return;
    }
    s_cur = s;               /* before collect(): the info provider looks at the current screen */
    ui_info_t i;
    collect(&i);
    draw(s, &i);
    int64_t now = esp_timer_get_time();
    s_off_at = timeout_s > 0 ? now + (int64_t)timeout_s * 1000000 : 0x7FFFFFFFFFFFFFFFLL;
    s_next_refresh = now + 1000000;
}

void ui_message(const char *line1, const char *line2, uint16_t color, int timeout_s)
{
    snprintf(s_msg1, sizeof(s_msg1), "%s", line1);
    snprintf(s_msg2, sizeof(s_msg2), "%s", line2 ? line2 : "");
    s_msg_color = color;
    ui_show(UI_MESSAGE, timeout_s);
}

void ui_service(int64_t now_us)
{
    if (!lcd_is_on()) {
        return;
    }
    if (now_us >= s_off_at) {
        ui_off();
        return;
    }
    if ((s_cur == UI_REC || s_cur == UI_STATUS || s_cur == UI_READY) && now_us >= s_next_refresh) {
        ui_info_t i;
        collect(&i);
        draw(s_cur, &i);
        s_next_refresh = now_us + 1000000;
    }
}

void ui_off(void)
{
    lcd_power_off();
    s_cur = UI_NONE;
}

bool ui_visible(void)
{
    return lcd_is_on();
}

ui_screen_t ui_current(void)
{
    return s_cur;
}

int64_t ui_off_deadline(void)
{
    return lcd_is_on() ? s_off_at : 0;
}
