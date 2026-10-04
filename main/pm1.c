#include "pm1.h"

#include "board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pm1";

#define REG_DEVICE_ID 0x00
#define REG_PWR_CFG   0x06
#define REG_I2C_CFG   0x09
#define REG_WDT_CNT   0x0A
#define REG_GPIO_MODE 0x10
#define REG_GPIO_OUT  0x11
#define REG_GPIO_DRV  0x13
#define REG_GPIO_FUNC0 0x16
#define REG_VBAT_L    0x22
#define REG_VIN_L     0x24

#define PWR_CFG_CHG_EN (1u << 0)
#define PM1_GPIO_LCD   2 /* L3B rail enable */
#define PM1_GPIO_PA    3 /* speaker amplifier enable, keep low */

#define USB_PRESENT_MV 4000

static i2c_master_dev_handle_t s_dev;
static bool s_ok;

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 50);
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(s_dev, b, sizeof(b), 50);
}

static esp_err_t update(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    esp_err_t err = rd(reg, &cur, 1);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t nv = (uint8_t)((cur & ~mask) | (val & mask));
    return nv == cur ? ESP_OK : wr(reg, nv);
}

/* 2 bit function field of GPIO n: 00 = plain GPIO */
static esp_err_t gpio_as_output(int n, bool high)
{
    esp_err_t err = update(REG_GPIO_FUNC0, (uint8_t)(3u << (n * 2)), 0);
    if (err == ESP_OK) err = update(REG_GPIO_DRV, (uint8_t)(1u << n), 0);          /* push-pull */
    if (err == ESP_OK) err = update(REG_GPIO_OUT, (uint8_t)(1u << n), high ? (uint8_t)(1u << n) : 0);
    if (err == ESP_OK) err = update(REG_GPIO_MODE, (uint8_t)(1u << n), (uint8_t)(1u << n)); /* output */
    return err;
}

esp_err_t pm1_init(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_PM1,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(board_i2c_bus(), &cfg, &s_dev);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t id = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = rd(REG_DEVICE_ID, &id, 1);   /* the first access may only wake the chip up */
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "M5PM1 not answering: %s", esp_err_to_name(err));
        return err;
    }
    s_ok = true;
    wr(REG_I2C_CFG, 0x00);   /* never let the PM1 doze off on an idle I2C bus */
    wr(REG_WDT_CNT, 0x00);   /* no PM1 watchdog */
    update(REG_PWR_CFG, PWR_CFG_CHG_EN, PWR_CFG_CHG_EN); /* charger on (bit may be cleared by a chip reset) */
    gpio_as_output(PM1_GPIO_PA, false);                  /* speaker amp off */
    ESP_LOGI(TAG, "M5PM1 id 0x%02x", id);
    return ESP_OK;
}

bool pm1_present(void)
{
    return s_ok;
}

esp_err_t pm1_lcd_power(bool on)
{
    if (!s_ok) {
        return ESP_ERR_INVALID_STATE;
    }
    return gpio_as_output(PM1_GPIO_LCD, on);
}

static uint16_t rd_mv(uint8_t reg)
{
    uint8_t b[2];
    if (!s_ok || rd(reg, b, 2) != ESP_OK) {
        return 0;
    }
    return (uint16_t)(b[0] | (b[1] << 8));
}

uint16_t pm1_battery_mv(void)
{
    return rd_mv(REG_VBAT_L);
}

uint16_t pm1_vin_mv(void)
{
    return rd_mv(REG_VIN_L);
}

bool pm1_usb_present(void)
{
    return pm1_vin_mv() >= USB_PRESENT_MV;
}

int pm1_battery_percent(uint16_t mv)
{
    /* rough single-cell Li-ion discharge curve under light load */
    static const struct { uint16_t mv; uint8_t pct; } curve[] = {
        {3300, 0}, {3500, 5}, {3600, 15}, {3700, 30}, {3750, 40}, {3800, 50},
        {3900, 65}, {4000, 80}, {4100, 92}, {4200, 100},
    };
    if (mv <= curve[0].mv) return 0;
    for (unsigned i = 1; i < sizeof(curve) / sizeof(curve[0]); i++) {
        if (mv <= curve[i].mv) {
            int span = curve[i].mv - curve[i - 1].mv;
            return curve[i - 1].pct + (curve[i].pct - curve[i - 1].pct) * (mv - curve[i - 1].mv) / span;
        }
    }
    return 100;
}
