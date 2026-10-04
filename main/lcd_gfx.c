/* Hardware independent drawing on the (rotated) frame buffer: pixels, rectangles, 8x8 text. */
#include <string.h>

#include "app_config.h"
#include "font8x8_basic.h"
#include "lcd.h"

uint16_t *lcd_gfx_fb;

int lcd_width(void)
{
    return (GYROLOG_UI_ROT & 1) ? LCD_NATIVE_H : LCD_NATIVE_W;
}

int lcd_height(void)
{
    return (GYROLOG_UI_ROT & 1) ? LCD_NATIVE_W : LCD_NATIVE_H;
}

/* logical (u right, v down, as the user reads it) -> native panel pixel index, -1 if outside */
static inline int native_index(int u, int v)
{
    int xn, yn;
    switch (GYROLOG_UI_ROT) {
    case 1:  xn = LCD_NATIVE_W - 1 - v; yn = u; break;                              /* USB-C to the right */
    case 2:  xn = LCD_NATIVE_W - 1 - u; yn = LCD_NATIVE_H - 1 - v; break;           /* USB-C at the top */
    case 3:  xn = v; yn = LCD_NATIVE_H - 1 - u; break;                              /* USB-C to the left */
    default: xn = u; yn = v; break;                                                 /* USB-C at the bottom */
    }
    if ((unsigned)xn >= LCD_NATIVE_W || (unsigned)yn >= LCD_NATIVE_H) {
        return -1;
    }
    return yn * LCD_NATIVE_W + xn;
}

static inline void put_px(int u, int v, uint16_t c)
{
    int i = native_index(u, v);
    if (i >= 0) {
        lcd_gfx_fb[i] = c;
    }
}

uint16_t lcd_pixel(int u, int v)
{
    int i = lcd_gfx_fb ? native_index(u, v) : -1;
    return i >= 0 ? lcd_gfx_fb[i] : 0;
}

void lcd_fill(uint16_t color)
{
    if (!lcd_gfx_fb) return;
    for (int i = 0; i < LCD_NATIVE_W * LCD_NATIVE_H; i++) {
        lcd_gfx_fb[i] = color;
    }
}

void lcd_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!lcd_gfx_fb) return;
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            put_px(x + i, y + j, color);
        }
    }
}

int lcd_text_width(const char *s, int scale)
{
    return (int)strlen(s) * 8 * scale;
}

int lcd_text(int x, int y, const char *s, int scale, uint16_t fg, uint16_t bg)
{
    if (!lcd_gfx_fb) return x;
    for (; *s; s++, x += 8 * scale) {
        const unsigned char *g = font8x8_basic[(unsigned char)*s & 0x7F];
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                uint16_t c = (g[row] & (1u << col)) ? fg : bg;
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        put_px(x + col * scale + dx, y + row * scale + dy, c);
                    }
                }
            }
        }
    }
    return x;
}

