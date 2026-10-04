#include "imu_fifo.h"

#include <string.h>

#define HDR_MASK       0xFCu
#define HDR_ACC        0x84u
#define HDR_GYR        0x88u
#define HDR_GYR_ACC    0x8Cu
#define HDR_AUX        0x90u
#define HDR_AUX_ACC    0x94u
#define HDR_AUX_GYR    0x98u
#define HDR_ALL        0x9Cu
#define HDR_SENSORTIME 0x44u
#define HDR_SKIP       0x40u
#define HDR_INPUT_CFG  0x48u
#define HDR_EMPTY      0x80u

static inline int16_t rd16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(p[0] | (p[1] << 8));
}

static void rd_axes(const uint8_t *p, int16_t *dst)
{
    dst[0] = rd16(p);
    dst[1] = rd16(p + 2);
    dst[2] = rd16(p + 4);
}

size_t imu_fifo_parse(const uint8_t *data, size_t len, bool want_acc, imu_sample_t *out, size_t out_max,
                      imu_fifo_info_t *info, size_t *consumed)
{
    size_t pos = 0, n = 0;
    imu_fifo_info_t scratch;
    if (!info) {
        info = &scratch;
    }
    while (pos < len && n < out_max) {
        uint8_t hdr = data[pos] & HDR_MASK;
        size_t body;
        bool has_aux = false, has_gyr = false, has_acc = false;
        switch (hdr) {
        case HDR_ACC:     body = 6;  has_acc = true; break;
        case HDR_GYR:     body = 6;  has_gyr = true; break;
        case HDR_GYR_ACC: body = 12; has_gyr = true; has_acc = true; break;
        case HDR_AUX:     body = 8;  has_aux = true; break;
        case HDR_AUX_ACC: body = 14; has_aux = true; has_acc = true; break;
        case HDR_AUX_GYR: body = 14; has_aux = true; has_gyr = true; break;
        case HDR_ALL:     body = 20; has_aux = true; has_gyr = true; has_acc = true; break;
        case HDR_SENSORTIME: body = 3; break;
        case HDR_SKIP:       body = 1; break;
        case HDR_INPUT_CFG:  body = 4; break;
        case HDR_EMPTY:
            /* over-read: everything after this is padding */
            *consumed = len;
            return n;
        default:
            info->invalid = true;
            *consumed = len;
            return n;
        }
        if (pos + 1 + body > len) {
            break; /* partial frame: leave it to the caller */
        }
        const uint8_t *p = data + pos + 1;
        if (hdr == HDR_SENSORTIME) {
            info->has_st = true;
            info->st24 = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
        } else if (hdr == HDR_SKIP) {
            info->skipped += p[0];
        } else if (hdr == HDR_INPUT_CFG) {
            /* nothing to do */
        } else {
            /* data frame; order inside the frame is aux, gyro, acc */
            const uint8_t *q = p + (has_aux ? 8 : 0);
            const uint8_t *g = has_gyr ? q : NULL;
            const uint8_t *a = has_acc ? q + (has_gyr ? 6 : 0) : NULL;
            if (g && (!want_acc || a)) {
                imu_sample_t *s = &out[n++];
                rd_axes(g, &s->v[0]);
                if (want_acc) {
                    rd_axes(a, &s->v[3]);
                } else {
                    memset(&s->v[3], 0, 3 * sizeof(int16_t));
                }
            } else {
                info->dropped++;
            }
        }
        pos += 1 + body;
    }
    *consumed = pos;
    return n;
}
