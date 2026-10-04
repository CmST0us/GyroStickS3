/*
 * ST7789P3 135x240 LCD of the StickS3: power sequencing, a small RGB565 frame buffer and
 * 8x8-font text drawing, with a logical rotation so the UI can be read whichever way the
 * stick is mounted.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define LCD_NATIVE_W 135   /* panel resolution, portrait, USB-C port at the bottom */
#define LCD_NATIVE_H 240

/* Frame buffer the drawing functions render into (native portrait layout, set by lcd.c / tests). */
extern uint16_t *lcd_gfx_fb;

/* colours are stored byte-swapped, as the panel wants them */
#define LCD_RGB(r, g, b) __builtin_bswap16((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define LCD_BLACK  LCD_RGB(0, 0, 0)
#define LCD_WHITE  LCD_RGB(255, 255, 255)
#define LCD_RED    LCD_RGB(255, 40, 40)
#define LCD_GREEN  LCD_RGB(40, 220, 80)
#define LCD_YELLOW LCD_RGB(255, 210, 0)
#define LCD_BLUE   LCD_RGB(60, 120, 255)
#define LCD_GREY   LCD_RGB(150, 150, 150)

/* Called repeatedly while the LCD code has to wait (lets the recorder keep draining the IMU FIFO). */
void lcd_set_wait_hook(void (*hook)(void));

esp_err_t lcd_power_on(void);
void      lcd_power_off(void);
bool      lcd_is_on(void);
/* Keep all LCD related pins low through deep sleep (call after lcd_power_off). */
void      lcd_hold_for_deep_sleep(void);
/* Release the pin hold after waking up from deep sleep. */
void      lcd_release_hold(void);

int  lcd_width(void);   /* logical size, depends on the configured rotation */
int  lcd_height(void);
void lcd_fill(uint16_t color);
void lcd_rect(int x, int y, int w, int h, uint16_t color);
/* Draw `s` with the 8x8 font scaled by `scale`; returns the x coordinate after the text. */
int  lcd_text(int x, int y, const char *s, int scale, uint16_t fg, uint16_t bg);
int  lcd_text_width(const char *s, int scale);
/* Read back a logical pixel (tests / previews). */
uint16_t lcd_pixel(int x, int y);
void lcd_flush(void);
