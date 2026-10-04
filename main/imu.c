#include "imu.h"

#include <string.h>

#include "app_config.h"
#include "bmi270.h"
#include "board.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

#define REG_ACC_X_LSB  0x0C
#define REG_SENSORTIME 0x18
#define REG_FIFO_LEN0  0x24
#define REG_FIFO_DATA  0x26
#define REG_ACC_CONF   0x40
#define REG_ACC_RANGE  0x41
#define REG_GYR_CONF   0x42
#define REG_GYR_RANGE  0x43
#define REG_FIFO_CFG0  0x48
#define REG_FIFO_CFG1  0x49
#define REG_PWR_CONF   0x7C
#define REG_PWR_CTRL   0x7D
#define REG_CMD        0x7E
#define CMD_FIFO_FLUSH 0xB0

#define FIFO_SIZE   2048
#define TIME_EXTRA  6          /* sensortime frame (4) + padding, read past the last valid frame */
#define RX_PREFIX   32         /* room to prepend a partial frame left over from the previous read */
#define RX_MAX      (FIFO_SIZE + TIME_EXTRA + 16)
#define FIFO_NEARLY_FULL (FIFO_SIZE - 64)

static struct bmi2_dev s_dev;
static i2c_master_dev_handle_t s_i2c;
static bool s_ready;
static uint8_t s_rx[RX_PREFIX + RX_MAX];
static size_t s_carry;

#ifdef CONFIG_GYROLOG_SIM_IMU
static int64_t s_sim_t0_us;
static uint64_t s_sim_frames;     /* frames the "sensor" has produced since start (including dropped ones) */
static uint64_t s_sim_read;       /* frames already delivered or dropped */
static bool s_sim_dropout_done;
static uint32_t s_sim_rng = 1;
#endif

static BMI2_INTF_RETURN_TYPE bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr)
{
    (void)ptr;
    return i2c_master_transmit_receive(s_i2c, &reg, 1, data, len, 100) == ESP_OK ? BMI2_INTF_RET_SUCCESS : -1;
}

static BMI2_INTF_RETURN_TYPE bus_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr)
{
    (void)ptr;
    uint8_t tmp[1 + 64];
    if (len > 64) {
        return -1;
    }
    tmp[0] = reg;
    memcpy(tmp + 1, data, len);
    return i2c_master_transmit(s_i2c, tmp, len + 1, 100) == ESP_OK ? BMI2_INTF_RET_SUCCESS : -1;
}

static void bus_delay_us(uint32_t us, void *ptr)
{
    (void)ptr;
    if (us < 2000) {
        esp_rom_delay_us(us);
    } else {
        vTaskDelay(pdMS_TO_TICKS(us / 1000 + 1));
    }
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t n)
{
    return bmi2_get_regs(reg, buf, (uint16_t)n, &s_dev) == BMI2_OK ? ESP_OK : ESP_FAIL;
}

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    return bmi2_set_regs(reg, &val, 1, &s_dev) == BMI2_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t imu_init(void)
{
#ifdef CONFIG_GYROLOG_SIM_IMU
    ESP_LOGW(TAG, "SIMULATED IMU");
    s_ready = true;
    return ESP_OK;
#endif
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_IMU,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(board_i2c_bus(), &cfg, &s_i2c);
    if (err != ESP_OK) {
        return err;
    }
    memset(&s_dev, 0, sizeof(s_dev));
    s_dev.intf = BMI2_I2C_INTF;
    s_dev.read = bus_read;
    s_dev.write = bus_write;
    s_dev.delay_us = bus_delay_us;
    s_dev.read_write_len = 32;
    s_dev.config_file_ptr = NULL; /* default 8 KiB blob compiled into bmi270.c */
    int8_t rslt = bmi270_init(&s_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMI270 chip id 0x%02x", s_dev.chip_id);
    s_ready = true;
    return imu_stop();
}

static uint8_t odr_code(int hz)
{
    switch (hz) {
    case 25: return 0x06;
    case 50: return 0x07;
    case 100: return 0x08;
    case 200: return 0x09;
    case 400: return 0x0A;
    case 800: return 0x0B;
    default: return 0x09;
    }
}

static uint8_t gyro_range_code(int dps)
{
    switch (dps) {
    case 2000: return 0;
    case 1000: return 1;
    case 500: return 2;
    case 250: return 3;
    default: return 4; /* 125 */
    }
}

static uint8_t acc_range_code(int g)
{
    switch (g) {
    case 2: return 0;
    case 4: return 1;
    case 8: return 2;
    default: return 3; /* 16 */
    }
}

esp_err_t imu_start(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
#ifdef CONFIG_GYROLOG_SIM_IMU
    s_sim_t0_us = esp_timer_get_time();
    s_sim_frames = 0;
    s_sim_read = 0;
    s_sim_dropout_done = false;
    s_carry = 0;
    return ESP_OK;
#endif
    const bool acc = GYROLOG_CHANNELS == 6;
    uint8_t odr = odr_code(GYROLOG_ODR_HZ);
    esp_err_t err = ESP_OK;
    /* acc: performance filter, normal bandwidth; gyro: performance filter (+ low noise) */
    err |= reg_write(REG_ACC_CONF, (uint8_t)(0x80 | 0x20 | odr));
    err |= reg_write(REG_ACC_RANGE, acc_range_code(GYROLOG_ACC_G));
#ifdef CONFIG_GYROLOG_IMU_LOW_NOISE
    err |= reg_write(REG_GYR_CONF, (uint8_t)(0x80 | 0x40 | 0x20 | odr));
#else
    err |= reg_write(REG_GYR_CONF, (uint8_t)(0x80 | 0x20 | odr));
#endif
    err |= reg_write(REG_GYR_RANGE, gyro_range_code(GYROLOG_GYRO_DPS));
    err |= reg_write(REG_FIFO_CFG0, 0x02);                                   /* sensor time frame on, overwrite when full */
    err |= reg_write(REG_FIFO_CFG1, (uint8_t)(0x10 | 0x80 | (acc ? 0x40 : 0))); /* header mode, gyro (+acc) */
    err |= reg_write(REG_PWR_CTRL, (uint8_t)(0x02 | (acc ? 0x04 : 0)));       /* gyro (+acc) on */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configuration failed");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(70)); /* gyro start-up time */
    err = reg_write(REG_CMD, CMD_FIFO_FLUSH);
    s_carry = 0;
    return err;
}

esp_err_t imu_stop(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_carry = 0;
#ifdef CONFIG_GYROLOG_SIM_IMU
    return ESP_OK;
#else
    return reg_write(REG_PWR_CTRL, 0x00);
#endif
}

#ifdef CONFIG_GYROLOG_SIM_IMU
/* ---- synthetic FIFO source (developer option): same byte stream the BMI270 would deliver ---- */

static uint32_t sim_rand(void)
{
    s_sim_rng = s_sim_rng * 1664525u + 1013904223u;
    return s_sim_rng >> 8;
}

static void put_le16(uint8_t *p, int v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static esp_err_t fifo_fetch(uint8_t *dst, size_t cap, size_t *got, bool *near_full)
{
    const bool acc = GYROLOG_CHANNELS == 6;
    const uint32_t tps = 25600 / GYROLOG_ODR_HZ;
    int64_t elapsed = esp_timer_get_time() - s_sim_t0_us;
    s_sim_frames = (uint64_t)elapsed * GYROLOG_ODR_HZ / 1000000ULL;
    if (!s_sim_dropout_done && elapsed > 8000000) { /* lose 150 ms once: the FIFO "overflowed" */
        uint64_t lost = (uint64_t)GYROLOG_ODR_HZ * 150 / 1000;
        if (s_sim_frames > s_sim_read + lost) {
            s_sim_read += lost;
            s_sim_dropout_done = true;
        }
    }
    size_t frame_len = acc ? 13 : 7;
    size_t max_frames = (cap - TIME_EXTRA) / frame_len;
    uint64_t n = s_sim_frames - s_sim_read;
    if (n > max_frames) {
        n = max_frames;
    }
    size_t pos = 0;
    for (uint64_t i = 0; i < n; i++) {
        double t = (double)(s_sim_read + i) / GYROLOG_ODR_HZ;
        dst[pos++] = acc ? 0x8C : 0x88;
        for (int c = 0; c < 3; c++) {
            double w = 3000.0 * __builtin_sin(6.2831853 * (0.5 + 0.7 * c) * t + c) + 400.0 * __builtin_sin(6.2831853 * 7.0 * t);
            put_le16(dst + pos, (int)w + 12 + (int)(sim_rand() % 7) - 3);
            pos += 2;
        }
        if (acc) {
            put_le16(dst + pos, (int)(300.0 * __builtin_sin(6.2831853 * 1.3 * t)) + (int)(sim_rand() % 5));
            put_le16(dst + pos + 2, (int)(200.0 * __builtin_sin(6.2831853 * 0.7 * t)) + (int)(sim_rand() % 5));
            put_le16(dst + pos + 4, 4096 + (int)(100.0 * __builtin_sin(6.2831853 * 2.9 * t)) + (int)(sim_rand() % 5));
            pos += 6;
        }
    }
    s_sim_read += n;
    uint32_t st = (uint32_t)(0xFFFFC0u + s_sim_read * tps) & 0xFFFFFFu; /* starts close to the 24 bit wrap */
    dst[pos++] = 0x44;
    dst[pos++] = (uint8_t)st;
    dst[pos++] = (uint8_t)(st >> 8);
    dst[pos++] = (uint8_t)(st >> 16);
    dst[pos++] = 0x80; /* over-read marker */
    dst[pos++] = 0x00;
    *got = pos;
    *near_full = false;
    return ESP_OK;
}
#else
static esp_err_t fifo_fetch(uint8_t *dst, size_t cap, size_t *got, bool *near_full)
{
    uint8_t lb[2];
    esp_err_t err = reg_read(REG_FIFO_LEN0, lb, 2);
    if (err != ESP_OK) {
        return err;
    }
    size_t avail = (size_t)(lb[0] | ((lb[1] & 0x3F) << 8));
    *got = 0;
    *near_full = avail >= FIFO_NEARLY_FULL;
    if (avail == 0) {
        return ESP_OK;
    }
    size_t want = avail + TIME_EXTRA; /* read past the last frame so the sensor time frame comes along */
    if (want > cap) {
        want = cap;
    }
    err = reg_read(REG_FIFO_DATA, dst, want);
    if (err == ESP_OK) {
        *got = want;
    }
    return err;
}
#endif

esp_err_t imu_read_burst(imu_sample_t *out, size_t max, size_t *n, imu_burst_t *info)
{
    memset(info, 0, sizeof(*info));
    *n = 0;
    uint8_t *dst = s_rx + RX_PREFIX;
    size_t got = 0;
    bool near_full = false;
    esp_err_t err = fifo_fetch(dst, RX_MAX, &got, &near_full);
    if (err != ESP_OK) {
        return err;
    }
    info->overflow = near_full;
    if (got == 0) {
        return ESP_OK;
    }
    info->fifo_bytes = (uint16_t)got;

    /* a partial frame left over from the previous read sits right in front of dst */
    uint8_t *start = dst - s_carry;
    size_t total = s_carry + got;
    imu_fifo_info_t fi;
    memset(&fi, 0, sizeof(fi));
    size_t used;
    *n = imu_fifo_parse(start, total, GYROLOG_CHANNELS == 6, out, max, &fi, &used);
    size_t left = total - used;
    if (left > RX_PREFIX) {
        left = 0; /* cannot happen with valid data; drop garbage */
    }
    if (left) {
        memmove(s_rx + RX_PREFIX - left, start + used, left);
    }
    s_carry = left;
    info->has_st = fi.has_st;
    info->st24 = fi.st24;
    info->skipped = fi.skipped;
    info->dropped = fi.dropped;
    if (fi.skipped) {
        info->overflow = true;
    }
#ifndef CONFIG_GYROLOG_SIM_IMU
    if (!fi.has_st) {
        /* No sensor time frame: fall back to reading the counter right now (a little later than the last sample). */
        uint8_t t[3];
        if (reg_read(REG_SENSORTIME, t, 3) == ESP_OK) {
            info->has_st = true;
            info->st24 = (uint32_t)t[0] | ((uint32_t)t[1] << 8) | ((uint32_t)t[2] << 16);
        }
    }
#endif
    return ESP_OK;
}

esp_err_t imu_peek_accel_g(float g[3])
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
#ifdef CONFIG_GYROLOG_SIM_IMU
    g[0] = 0.0f; g[1] = 0.0f; g[2] = 1.0f;
    return ESP_OK;
#endif
    const float lsb_per_g = 32768.0f / (float)GYROLOG_ACC_G;
    esp_err_t err = reg_write(REG_ACC_CONF, (uint8_t)(0x80 | 0x20 | 0x08)); /* 100 Hz performance */
    err |= reg_write(REG_ACC_RANGE, acc_range_code(GYROLOG_ACC_G));
    err |= reg_write(REG_PWR_CTRL, 0x04);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    uint8_t b[6];
    err = reg_read(REG_ACC_X_LSB, b, 6);
    reg_write(REG_PWR_CTRL, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 3; i++) {
        g[i] = (float)(int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / lsb_per_g;
    }
    return ESP_OK;
}
