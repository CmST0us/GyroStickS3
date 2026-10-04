/*
 * StickS3 Gyroflow logger
 *
 *   main button  click       start / stop recording
 *   main button  long press  deep sleep / wake
 *   aux button   click       status screen (battery, storage, live accelerometer for axis checks)
 *   aux button   long press  erase menu (main click = yes, aux click = no)
 *
 * Everything runs in one task. While recording the CPU sleeps in light sleep between BMI270 FIFO
 * reads; the screen is only powered for a few seconds after a button press.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "app_config.h"
#include "board.h"
#include "buttons.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gyl_format.h"
#include "hostlink.h"
#include "imu.h"
#include "lcd.h"
#include "logstore.h"
#include "pm1.h"
#include "power.h"
#include "recorder.h"
#include "ui.h"

static const char *TAG = "app";

typedef enum { APP_IDLE, APP_RECORDING } app_state_t;

#define US(x) ((int64_t)(x) * 1000000LL)

static app_state_t s_state = APP_IDLE;
static int64_t s_rec_start_us;
static int64_t s_next_fifo_us;
static int64_t s_next_batt_us;
static int64_t s_next_usb_us;
static int64_t s_last_activity_us;
static uint16_t s_batt_mv;
static int s_low_batt_hits;
static bool s_usb;
static bool s_imu_ok, s_pm1_ok;
static uint32_t s_last_rec_seconds, s_last_rec_bytes;
static imu_sample_t s_samples[320];

/* Bytes per recorded second observed in the last recording, kept over deep sleep. */
static RTC_DATA_ATTR float s_rate_bps;

static float nominal_rate_bps(void)
{
    return (float)GYROLOG_ODR_HZ * (GYROLOG_CHANNELS == 6 ? 4.2f : 2.2f);
}

/* ---------------------------------------------------------------------------------------- UI info */
static void provide_info(ui_info_t *i)
{
    const logstore_info_t *li = logstore_info();
    const rec_status_t *rs = rec_status();
    int64_t now = esp_timer_get_time();
    i->recording = s_state == APP_RECORDING;
    i->usb = s_usb;
    i->batt_mv = s_batt_mv;
    i->batt_pct = pm1_battery_percent(s_batt_mv);
    i->total_pages = li->total_pages;
    i->free_pages = li->next_page < li->total_pages ? li->total_pages - li->next_page : 0;
    i->sessions = li->sessions;
    i->imu_ok = s_imu_ok;
    i->pm1_ok = s_pm1_ok;
    if (i->recording) {
        i->rec_samples = rs->samples;
        i->rec_seconds = (uint32_t)((now - s_rec_start_us) / 1000000);
        i->rec_bytes = rs->bytes;
    } else {
        i->rec_seconds = s_last_rec_seconds;
        i->rec_bytes = s_last_rec_bytes;
    }
    float rate = s_rate_bps > 50.0f ? s_rate_bps : nominal_rate_bps();
    if (i->recording && i->rec_seconds >= 20 && rs->bytes > 0) {
        rate = (float)rs->bytes / (float)i->rec_seconds;
    }
    i->free_seconds = (uint32_t)((float)i->free_pages * (float)GYL_PAYLOAD_MAX / rate);
    if (!i->recording && ui_current() == UI_STATUS && s_imu_ok) {
        i->has_acc = imu_peek_accel_g(i->acc_g) == ESP_OK;
    }
}

/* ------------------------------------------------------------------------------------- recording */
static void service_recording(void)
{
    if (s_state != APP_RECORDING) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now < s_next_fifo_us) {
        return;
    }
    s_next_fifo_us += (int64_t)FIFO_POLL_MS * 1000;
    if (s_next_fifo_us < now) {
        s_next_fifo_us = now + (int64_t)FIFO_POLL_MS * 1000;
    }
    size_t n;
    imu_burst_t b;
    if (imu_read_burst(s_samples, sizeof(s_samples) / sizeof(s_samples[0]), &n, &b) == ESP_OK) {
        rec_feed(s_samples, n, &b, s_batt_mv);
    }
}

static void update_battery(int64_t now)
{
    uint16_t mv = pm1_battery_mv();
    if (mv) {
        s_batt_mv = mv;
    }
    s_next_batt_us = now + US(s_state == APP_RECORDING ? 30 : 20);
}

static void update_usb(int64_t now)
{
    s_usb = pm1_usb_present();
    s_next_usb_us = now + US(2);
}

static void go_to_sleep(void) __attribute__((noreturn));
static void stop_recording(const char *why);

static void start_recording(void)
{
    const logstore_info_t *li = logstore_info();
    if (!s_imu_ok) {
        ui_message("IMU ERR", "CHECK SENSOR", LCD_RED, 4);
        return;
    }
    if (li->next_page >= li->total_pages) {
        ui_message("FULL", "ERASE THE LOGS", LCD_RED, 4);
        return;
    }
    if (s_batt_mv && s_batt_mv < LOW_BATT_MV && !s_usb) {
        ui_message("LOW BATT", "CHARGE FIRST", LCD_RED, 4);
        return;
    }
    if (imu_start() != ESP_OK || rec_begin() != ESP_OK) {
        imu_stop();
        ui_message("START ERR", "", LCD_RED, 4);
        return;
    }
    int64_t now = esp_timer_get_time();
    s_state = APP_RECORDING;
    s_rec_start_us = now;
    s_next_fifo_us = now + (int64_t)FIFO_POLL_MS * 1000;
    s_next_batt_us = now + US(30);
    ESP_LOGI(TAG, "recording");
    ui_show(UI_REC, REC_SCREEN_S);
}

static void stop_recording(const char *why)
{
    if (s_state != APP_RECORDING) {
        return;
    }
    s_next_fifo_us = 0;                  /* force one last read to drain the FIFO */
    service_recording();
    imu_stop();
    int64_t now = esp_timer_get_time();
    uint32_t secs = (uint32_t)((now - s_rec_start_us) / 1000000);
    esp_err_t err = rec_end();
    const rec_status_t *rs = rec_status();
    s_last_rec_seconds = secs;
    s_last_rec_bytes = rs->bytes;
    if (secs >= 20 && rs->bytes) {
        s_rate_bps = (float)rs->bytes / (float)secs;
    }
    s_state = APP_IDLE;
    s_last_activity_us = now;
    logstore_init();                     /* refresh counters */
    ESP_LOGI(TAG, "stopped (%s): %u s", why, (unsigned)secs);
    if (rs->full) {
        ui_message("FULL", "STORAGE FULL", LCD_RED, 6);
    } else if (err != ESP_OK) {
        ui_message("FLASH ERR", "LOG MAY BE CUT", LCD_RED, 6);
    } else if (strcmp(why, "sleep") != 0) {
        ui_show(UI_SAVED, REC_SCREEN_S);
    }
}

static void go_to_sleep(void)
{
    if (s_state == APP_RECORDING) {
        stop_recording("sleep");
    }
    ui_show(UI_SLEEP, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    power_deep_sleep();
}

static void do_erase(void)
{
    ui_message("ERASING", "PLEASE WAIT", LCD_YELLOW, 0);
    esp_err_t err = logstore_erase_all();
    if (err == ESP_OK) {
        ui_message("ERASED", "LOG IS EMPTY", LCD_GREEN, 3);
    } else {
        ui_message("ERASE ERR", "", LCD_RED, 4);
    }
}

#ifdef CONFIG_GYROLOG_SELFTEST
/* Scripted run for automated tests (QEMU): two recordings, then a summary line. */
static void selftest_step(int64_t now)
{
    static int64_t t0;
    static int phase;
    if (!t0) {
        t0 = now;
    }
    int64_t t = (now - t0) / 1000000;
    switch (phase) {
    case 0:
        if (t >= 2) { ESP_LOGW(TAG, "SELFTEST: start #1"); start_recording(); phase++; }
        break;
    case 1:
        if (t >= 22) { stop_recording("button"); ESP_LOGW(TAG, "SELFTEST: stop #1"); phase++; }
        break;
    case 2:
        if (t >= 24) { ESP_LOGW(TAG, "SELFTEST: start #2"); start_recording(); phase++; }
        break;
    case 3:
        if (t >= 36) { stop_recording("button"); ESP_LOGW(TAG, "SELFTEST: stop #2"); phase++; }
        break;
    case 4:
        if (t >= 38) {
            const logstore_info_t *li = logstore_info();
            ESP_LOGW(TAG, "SELFTEST: done, %u pages used, %u recordings", (unsigned)li->next_page, (unsigned)li->sessions);
            phase++;
        }
        break;
    default:
        break;
    }
}
#endif

/* ------------------------------------------------------------------------------------ main loop */
/* Block until `deadline_us` (or an earlier event). Light sleeps when nothing needs the CPU. */
static void wait_until(int64_t deadline_us, bool can_light_sleep)
{
    int64_t now = esp_timer_get_time();
    if (buttons_busy(now) || ui_visible() || s_usb) {
        vTaskDelay(1);                       /* 10 ms: keep buttons / screen / USB responsive */
        return;
    }
    int64_t us = deadline_us - now;
    if (us < 4000) {
        vTaskDelay(1);
        return;
    }
#ifdef CONFIG_GYROLOG_SELFTEST
    can_light_sleep = false; /* QEMU cannot light sleep */
#endif
    if (!can_light_sleep) {
        vTaskDelay(pdMS_TO_TICKS(us / 1000) ? pdMS_TO_TICKS(us / 1000) : 1);
        return;
    }
    esp_sleep_enable_timer_wakeup((uint64_t)us);
    buttons_arm_light_sleep_wake();
    esp_light_sleep_start();
}

static void main_loop(void)
{
    for (;;) {
        int64_t now = esp_timer_get_time();

        btn_event_t m = buttons_poll(BTN_MAIN, now);
        btn_event_t a = buttons_poll(BTN_AUX, now);
        if (m != BTN_EV_NONE || a != BTN_EV_NONE) {
            s_last_activity_us = now;
        }
        if (m == BTN_EV_CLICK) {
            if (ui_current() == UI_ERASE_CONFIRM && s_state == APP_IDLE) {
                do_erase();
            } else if (s_state == APP_RECORDING) {
                stop_recording("button");
            } else {
                start_recording();
            }
        } else if (m == BTN_EV_LONG) {
            go_to_sleep();
        }
        if (a == BTN_EV_CLICK) {
            if (ui_current() == UI_ERASE_CONFIRM) {
                ui_show(UI_READY, REC_SCREEN_S);
            } else {
                ui_show(UI_STATUS, 6);
            }
        } else if (a == BTN_EV_LONG && s_state == APP_IDLE) {
            ui_show(UI_ERASE_CONFIRM, 8);
        }

#ifdef CONFIG_GYROLOG_SELFTEST
        selftest_step(now);
#endif
        service_recording();
        if (s_state == APP_RECORDING) {
            const rec_status_t *rs = rec_status();
            if (rs->full) {
                stop_recording("full");
            } else if (rs->error) {
                stop_recording("error");
            }
        }

        now = esp_timer_get_time();
        if (now >= s_next_batt_us) {
            update_battery(now);
            if (s_batt_mv && s_batt_mv < LOW_BATT_MV && !s_usb) {
                if (++s_low_batt_hits >= 2) {
                    if (s_state == APP_RECORDING) {
                        stop_recording("lowbatt");
                    }
                    ui_message("LOW BATT", "SLEEPING NOW", LCD_RED, 0);
                    vTaskDelay(pdMS_TO_TICKS(2500));
                    power_deep_sleep();
                }
            } else {
                s_low_batt_hits = 0;
            }
        }
        if (now >= s_next_usb_us) {
            update_usb(now);
        }
        hostlink_poll();
        ui_service(now);

        /* idle timeout -> deep sleep (never while recording, powered from USB, or talking to the host) */
        bool host = hostlink_session_active(now);
        if (s_state == APP_IDLE && IDLE_SLEEP_S > 0 && !s_usb && !host && !buttons_busy(now) &&
            now - s_last_activity_us >= US(IDLE_SLEEP_S)) {
            go_to_sleep();
        }

        /* when to wake up next */
        int64_t deadline = now + US(1);
        if (s_state == APP_RECORDING && s_next_fifo_us < deadline) {
            deadline = s_next_fifo_us;
        }
        if (s_state == APP_IDLE && IDLE_SLEEP_S > 0 && !s_usb && !host) {
            int64_t t = s_last_activity_us + US(IDLE_SLEEP_S);
            if (t < deadline) deadline = t;
        }
        if (s_next_batt_us < deadline) deadline = s_next_batt_us;
        bool can_ls = !host && !s_usb && (s_state != APP_RECORDING || LIGHT_SLEEP_REC);
        wait_until(deadline, can_ls);
    }
}

void app_main(void)
{
    int64_t boot_us = esp_timer_get_time();
    power_after_wake();
    buttons_init();

    if (power_woke_by_button()) {
        /* A bump or a short tap must not turn the device on: require the long press to be completed. */
        int64_t need_us = (int64_t)LONG_PRESS_MS * 1000 - 250000 - boot_us; /* ~250 ms were spent in ROM + bootloader */
        int64_t t0 = esp_timer_get_time();
        for (;;) {
            if (!buttons_pressed(BTN_MAIN)) {
                power_deep_sleep();
            }
            if (esp_timer_get_time() - t0 >= need_us) {
                break;
            }
            vTaskDelay(1);
        }
        buttons_swallow_until_release(BTN_MAIN);
    }

    ESP_LOGI(TAG, "%s  %d Hz  %d ch  orientation %s", FW_VERSION, GYROLOG_ODR_HZ, GYROLOG_CHANNELS, GYROLOG_ORIENTATION);
    s_pm1_ok = pm1_init() == ESP_OK;
    s_imu_ok = imu_init() == ESP_OK;
    if (logstore_init() != ESP_OK) {
        ESP_LOGE(TAG, "log storage unavailable");
    }
    hostlink_init();
    lcd_set_wait_hook(service_recording);
    ui_set_info_provider(provide_info);

    int64_t now = esp_timer_get_time();
    s_last_activity_us = now;
    update_battery(now);
    update_usb(now);

    if (s_batt_mv && s_batt_mv < LOW_BATT_MV && !s_usb) {
        ui_message("LOW BATT", "CHARGE ME", LCD_RED, 3);
    } else if (!s_imu_ok) {
        ui_message("IMU ERR", "CHECK SENSOR", LCD_RED, 5);
    } else {
        ui_show(UI_READY, 8);
    }
    main_loop();
}
