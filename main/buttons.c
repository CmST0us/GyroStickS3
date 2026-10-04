#include "buttons.h"

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    gpio_num_t gpio;
    btn_state_t st;
} btn_t;

static btn_t s_btn[BTN_COUNT] = {
    [BTN_MAIN] = {.gpio = (gpio_num_t)BTN_MAIN_GPIO},
    [BTN_AUX] = {.gpio = (gpio_num_t)BTN_AUX_GPIO},
};

int buttons_gpio(btn_id_t id)
{
    return s_btn[id].gpio;
}

void buttons_init(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        gpio_reset_pin(s_btn[i].gpio);
        gpio_set_direction(s_btn[i].gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(s_btn[i].gpio, GPIO_PULLUP_ONLY);
        s_btn[i].st = (btn_state_t){0};
    }
}

bool buttons_pressed(btn_id_t id)
{
#ifdef CONFIG_GYROLOG_SELFTEST
    return false; /* QEMU has no pull-ups: inputs read low */
#else
    return gpio_get_level(s_btn[id].gpio) == 0;
#endif
}

void buttons_swallow_until_release(btn_id_t id)
{
    btn_swallow(&s_btn[id].st, buttons_pressed(id));
}

btn_event_t buttons_poll(btn_id_t id, int64_t now_us)
{
    return btn_step(&s_btn[id].st, buttons_pressed(id), now_us, (int64_t)LONG_PRESS_MS * 1000);
}

bool buttons_busy(int64_t now_us)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (btn_busy(&s_btn[i].st, now_us)) {
            return true;
        }
    }
    return false;
}

void buttons_wait_release(btn_id_t id, int timeout_ms)
{
    int stable_ms = 0;
    while (timeout_ms > 0) {
        if (buttons_pressed(id)) {
            stable_ms = 0;
        } else if ((stable_ms += 10) >= 50) {
            return;
        }
        vTaskDelay(1);
        timeout_ms -= 10;
    }
}

void buttons_arm_light_sleep_wake(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        gpio_wakeup_enable(s_btn[i].gpio, GPIO_INTR_LOW_LEVEL);
    }
    esp_sleep_enable_gpio_wakeup();
}
