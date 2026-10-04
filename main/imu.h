/* BMI270 driver: Bosch API for chip init, own register code + FIFO streaming. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "imu_fifo.h"

typedef struct {
    bool     has_st;    /* sensor time (24 bit) valid for the end of this burst */
    uint32_t st24;
    bool     overflow;  /* FIFO was (nearly) full: samples were probably lost */
    uint32_t skipped;   /* skip frames reported by the sensor */
    uint32_t dropped;   /* unusable frames */
    uint16_t fifo_bytes;/* bytes read */
} imu_burst_t;

/* Attach to the chip, upload the configuration blob, leave all sensors off. */
esp_err_t imu_init(void);
/* Configure ODR / ranges from Kconfig, enable the sensors and start filling the FIFO. */
esp_err_t imu_start(void);
/* Back to suspend (a few uA). */
esp_err_t imu_stop(void);
/* Read and parse everything currently in the FIFO. */
esp_err_t imu_read_burst(imu_sample_t *out, size_t max, size_t *n, imu_burst_t *info);
/* One-shot accelerometer reading in g (brings the accelerometer up for ~30 ms). */
esp_err_t imu_peek_accel_g(float g[3]);
