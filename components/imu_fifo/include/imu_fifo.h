/*
 * BMI270 FIFO frame parser (header mode, gyro + optional accel, sensor time frame).
 *
 * Pure C, no hardware access, so it can be unit tested on the host.
 *
 * Frame layout (BMI270 datasheet / Bosch SensorAPI bmi2.c):
 *   0x8C  gyro+acc frame   header + gyro xyz (3 x int16 LE) + acc xyz (3 x int16 LE)
 *   0x88  gyro frame       header + gyro xyz
 *   0x84  acc frame        header + acc xyz
 *   0x44  sensor time      header + 24 bit LE tick counter (39.0625 us / tick)
 *   0x40  skip frame       header + 1 byte (number of skipped frames)
 *   0x48  input config     header + 4 bytes
 *   0x80  over-read marker (FIFO empty): end of valid data
 * The two low bits of the header are interrupt tags and are ignored.
 */
#ifndef IMU_FIFO_H
#define IMU_FIFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t v[6]; /* gx gy gz ax ay az */
} imu_sample_t;

typedef struct {
    bool     has_st;   /* a sensor time frame was seen */
    uint32_t st24;     /* last sensor time value (24 bit) */
    uint32_t skipped;  /* frames the sensor reports as skipped */
    uint32_t dropped;  /* frames that lacked a required channel and were discarded */
    bool     invalid;  /* unknown header / garbage encountered, rest of the buffer ignored */
} imu_fifo_info_t;

/*
 * Parse as many complete frames as possible.
 *   want_acc : require accel data in each sample (6 channel mode)
 *   consumed : bytes fully processed. A trailing partial frame is not consumed; the BMI270 re-sends a frame that
 *              was cut off by the end of a read from its header on the next read, so the caller drops it.
 * Returns the number of samples written to `out`. Stops early if `out` is full.
 */
size_t imu_fifo_parse(const uint8_t *data, size_t len, bool want_acc, imu_sample_t *out, size_t out_max,
                      imu_fifo_info_t *info, size_t *consumed);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FIFO_H */
