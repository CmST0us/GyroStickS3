/* Host-side decoder (tests / tools). Not part of the firmware build. */
#include "gyl_codec.h"

#include <string.h>

void gyl_dec_init(gyl_dec_t *d, const uint8_t *buf, size_t len, uint8_t channels)
{
    memset(d, 0, sizeof(*d));
    d->buf = buf;
    d->len = len;
    d->channels = channels;
    for (unsigned i = 0; i < GYL_MAX_CH; i++) {
        d->mean16[i] = GYL_MEAN_INIT16;
    }
}

static bool get_bit(gyl_dec_t *d, uint32_t *bit)
{
    if (d->nbits == 0) {
        if (d->pos >= d->len) {
            return false;
        }
        d->acc = d->buf[d->pos++];
        d->nbits = 8;
    }
    d->nbits--;
    *bit = (d->acc >> d->nbits) & 1u;
    return true;
}

static bool get_bits(gyl_dec_t *d, uint32_t n, uint32_t *val)
{
    uint32_t v = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t b;
        if (!get_bit(d, &b)) {
            return false;
        }
        v = (v << 1) | b;
    }
    *val = v;
    return true;
}

bool gyl_dec_get(gyl_dec_t *d, int16_t *v)
{
    if (d->count == 0) {
        if (d->len - d->pos < (size_t)d->channels * 2u) {
            return false;
        }
        for (unsigned c = 0; c < d->channels; c++) {
            v[c] = (int16_t)(d->buf[d->pos] | (d->buf[d->pos + 1] << 8));
            d->pos += 2;
            d->prev[c] = v[c];
            d->prev2[c] = v[c];
        }
        d->count = 1;
        return true;
    }
    for (unsigned c = 0; c < d->channels; c++) {
        uint32_t k = gyl_rice_k(d->mean16[c]);
        uint32_t q = 0, b, u, r;
        for (;;) {
            if (!get_bit(d, &b)) {
                return false;
            }
            if (!b) {
                break;
            }
            if (++q == GYL_ESC_Q) {
                break;
            }
        }
        if (q == GYL_ESC_Q) {
            if (!get_bits(d, GYL_ESC_BITS, &u)) {
                return false;
            }
        } else {
            if (!get_bits(d, k, &r)) {
                return false;
            }
            u = (q << k) | r;
        }
        int32_t delta = (int32_t)(u >> 1) ^ -(int32_t)(u & 1u);
        int32_t pred = gyl_predict(d->prev[c], d->prev2[c], d->err1[c], d->err2[c]);
        int16_t x = (int16_t)(pred + delta);
        GYL_ADAPT(d, c, x, u);
        v[c] = x;
    }
    d->count++;
    return true;
}
