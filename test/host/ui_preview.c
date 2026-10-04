/* Renders every on-device screen with the real ui.c / lcd_gfx.c into PPM files (as the user sees them). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcd.h"
#include "ui.h"

static int64_t g_now = 1000000;
int64_t esp_timer_get_time(void) { return g_now; }

static int g_shot;
static const char *g_prefix = "ui";

esp_err_t lcd_power_on(void)
{
    if (!lcd_gfx_fb) {
        lcd_gfx_fb = calloc(LCD_NATIVE_W * LCD_NATIVE_H, sizeof(uint16_t));
    }
    return ESP_OK;
}
void lcd_power_off(void) { free(lcd_gfx_fb); lcd_gfx_fb = NULL; }
bool lcd_is_on(void) { return lcd_gfx_fb != NULL; }
void lcd_set_wait_hook(void (*hook)(void)) { (void)hook; }
void lcd_hold_for_deep_sleep(void) {}
void lcd_release_hold(void) {}

void lcd_flush(void)
{
    char name[64];
    snprintf(name, sizeof(name), "%s_%02d.ppm", g_prefix, g_shot++);
    FILE *f = fopen(name, "wb");
    int w = lcd_width(), h = lcd_height();
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint16_t c = __builtin_bswap16(lcd_pixel(x, y));
            fputc((c >> 11) << 3, f);
            fputc(((c >> 5) & 0x3F) << 2, f);
            fputc((c & 0x1F) << 3, f);
        }
    }
    fclose(f);
}

static ui_info_t g_info;
static void provider(ui_info_t *i) { *i = g_info; }

int main(int argc, char **argv)
{
    if (argc > 1) g_prefix = argv[1];
    ui_set_info_provider(provider);
    g_info = (ui_info_t){.batt_mv = 3912, .batt_pct = 68, .free_pages = 1850, .total_pages = 1904, .sessions = 3,
                         .imu_ok = true, .pm1_ok = true, .free_seconds = 8 * 3600 + 25 * 60, .usb = false};
    ui_show(UI_READY, 8);
    g_info.recording = true; g_info.rec_seconds = 3723; g_info.rec_bytes = 3100 * 1024; g_info.free_pages = 1100;
    g_info.free_seconds = 2 * 3600 + 5 * 60;
    ui_show(UI_REC, 3);
    g_info.recording = false; g_info.usb = true;
    ui_show(UI_SAVED, 3);
    g_info.has_acc = true; g_info.acc_g[0] = 0.02f; g_info.acc_g[1] = -0.01f; g_info.acc_g[2] = 0.99f;
    ui_show(UI_STATUS, 6);
    ui_show(UI_ERASE_CONFIRM, 8);
    ui_message("LOW BATT", "SLEEPING NOW", LCD_RED, 0);
    ui_message("FULL", "STORAGE FULL", LCD_RED, 6);
    g_info.free_pages = 31; g_info.free_seconds = 60; g_info.batt_mv = 3310; g_info.batt_pct = 0;
    ui_show(UI_READY, 8);
    ui_show(UI_SLEEP, 0);
    return 0;
}
