/* Turns IMU bursts into compressed pages and appends them to the log store. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "imu.h"

typedef struct {
    bool     active;
    bool     full;     /* log partition exhausted, recording must stop */
    bool     error;    /* flash error, recording must stop */
    uint32_t session_id;
    uint32_t samples;  /* samples recorded in this session */
    uint32_t pages;    /* pages written in this session */
    uint32_t gaps;     /* detected discontinuities */
    uint32_t bytes;    /* payload bytes written in this session */
} rec_status_t;

/* Start a new session. The IMU must already be streaming. */
esp_err_t rec_begin(void);
/* Feed one FIFO burst. May write one or more pages (blocks for ~60 ms per page). */
void rec_feed(const imu_sample_t *s, size_t n, const imu_burst_t *b, uint16_t batt_mv);
/* Flush the partial page and close the session. */
esp_err_t rec_end(void);
const rec_status_t *rec_status(void);
