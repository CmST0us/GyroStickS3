#include "power.h"

#include "app_config.h"
#include "buttons.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "imu.h"
#include "lcd.h"
#include "ui.h"

static const char *TAG = "power";

bool power_woke_by_button(void)
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
}

void power_after_wake(void)
{
    lcd_release_hold();
    if (rtc_gpio_is_valid_gpio((gpio_num_t)BTN_MAIN_GPIO)) {
        rtc_gpio_deinit((gpio_num_t)BTN_MAIN_GPIO);
    }
}

void power_deep_sleep(void)
{
    ESP_LOGI(TAG, "deep sleep");
    ui_off();
    imu_stop();
    /* A button still held would immediately wake us again. */
    buttons_wait_release(BTN_MAIN, 6000);
    lcd_hold_for_deep_sleep();

    gpio_num_t g = (gpio_num_t)BTN_MAIN_GPIO;
    rtc_gpio_init(g);
    rtc_gpio_set_direction(g, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(g);
    rtc_gpio_pullup_en(g);
    esp_sleep_enable_ext1_wakeup_io(1ULL << BTN_MAIN_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
    for (;;) {
    }
}
