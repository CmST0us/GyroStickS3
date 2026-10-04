#include "lcd.h"

#include <string.h>

#include "app_config.h"
#include "board.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pm1.h"

static const char *TAG = "lcd";

#define LCD_HOST SPI2_HOST

static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb;
static bool s_on;
static SemaphoreHandle_t s_done;
static void (*s_hook)(void);

void lcd_set_wait_hook(void (*hook)(void))
{
    s_hook = hook;
}

/* Wait `ms`, keeping the hook running. */
static void wait_ms(int ms)
{
    while (ms > 0) {
        int step = ms > 10 ? 10 : ms;
        vTaskDelay(pdMS_TO_TICKS(step) ? pdMS_TO_TICKS(step) : 1);
        ms -= step;
        if (s_hook) {
            s_hook();
        }
    }
}

static bool trans_done_cb(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    (void)io; (void)edata; (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_done, &woken);
    return woken == pdTRUE;
}

static void backlight_set(int pct)
{
    if (pct <= 0) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        return;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)(255 * pct / 100));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    ledc_channel_config_t c = {
        .gpio_num = PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&c);
}

static const gpio_num_t k_pins[] = {PIN_LCD_MOSI, PIN_LCD_SCLK, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST, PIN_LCD_BL};

static void pins_low(void)
{
    for (unsigned i = 0; i < sizeof(k_pins) / sizeof(k_pins[0]); i++) {
        gpio_reset_pin(k_pins[i]);
        gpio_set_direction(k_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(k_pins[i], 0);
        gpio_sleep_sel_dis(k_pins[i]); /* keep driving low in light sleep instead of floating (backlight gate!) */
    }
}

esp_err_t lcd_power_on(void)
{
    if (s_on) {
        return ESP_OK;
    }
    if (!s_done) {
        s_done = xSemaphoreCreateBinary();
    }
    s_fb = heap_caps_malloc(LCD_NATIVE_W * LCD_NATIVE_H * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb, 0, LCD_NATIVE_W * LCD_NATIVE_H * sizeof(uint16_t));
    lcd_gfx_fb = s_fb;

    pm1_lcd_power(true);
    wait_ms(100); /* rail settling, as in M5's own init */

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_NATIVE_W * LCD_NATIVE_H * 2,
    };
    esp_err_t err = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        goto fail;
    }
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 2,
        .on_color_trans_done = trans_done_cb,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &s_io);
    if (err != ESP_OK) {
        goto fail_bus;
    }
    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(s_io, &pcfg, &s_panel);
    if (err != ESP_OK) {
        goto fail_io;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, LCD_OFFSET_X, LCD_OFFSET_Y);
    esp_lcd_panel_disp_on_off(s_panel, true);

    backlight_init();
    s_on = true;
    lcd_fill(LCD_BLACK);
    lcd_flush();
    backlight_set(LCD_BRIGHTNESS_PCT);
    return ESP_OK;

fail_io:
    esp_lcd_panel_io_del(s_io);
fail_bus:
    spi_bus_free(LCD_HOST);
fail:
    ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));
    free(s_fb);
    s_fb = NULL;
    lcd_gfx_fb = NULL;
    pins_low();
    pm1_lcd_power(false);
    return err;
}

void lcd_power_off(void)
{
    if (!s_on) {
        return;
    }
    backlight_set(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    esp_lcd_panel_del(s_panel);
    esp_lcd_panel_io_del(s_io);
    spi_bus_free(LCD_HOST);
    s_panel = NULL;
    s_io = NULL;
    free(s_fb);
    s_fb = NULL;
    lcd_gfx_fb = NULL;
    s_on = false;
    pins_low();            /* do not back-feed the unpowered panel */
    pm1_lcd_power(false);
}

bool lcd_is_on(void)
{
    return s_on;
}

void lcd_hold_for_deep_sleep(void)
{
    pins_low();
    for (unsigned i = 0; i < sizeof(k_pins) / sizeof(k_pins[0]); i++) {
        gpio_hold_en(k_pins[i]);
    }
    gpio_deep_sleep_hold_en();
}

void lcd_release_hold(void)
{
    gpio_deep_sleep_hold_dis();
    for (unsigned i = 0; i < sizeof(k_pins) / sizeof(k_pins[0]); i++) {
        gpio_hold_dis(k_pins[i]);
    }
}

void lcd_flush(void)
{
    if (!s_on) return;
    xSemaphoreTake(s_done, 0); /* clear a stale give */
    if (esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_NATIVE_W, LCD_NATIVE_H, s_fb) == ESP_OK) {
        xSemaphoreTake(s_done, pdMS_TO_TICKS(200));
    }
}
