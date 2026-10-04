#include "gyl_codec.h"

#include <string.h>

static inline uint32_t zigzag(int32_t d)
{
    return ((uint32_t)d << 1) ^ (uint32_t)(d >> 31);
}

static inline void put_bits(gyl_enc_t *e, uint32_t val, uint32_t n)
{
    /* nbits < 8 on entry, n <= 17  ->  fits the 32 bit accumulator */
    e->acc = (e->acc << n) | (val & ((1u << n) - 1u));
    e->nbits += n;
    while (e->nbits >= 8u) {
        e->buf[e->pos++] = (uint8_t)(e->acc >> (e->nbits - 8u));
        e->nbits -= 8u;
    }
}

void gyl_enc_init(gyl_enc_t *e, uint8_t *buf, size_t cap, uint8_t channels)
{
    memset(e, 0, sizeof(*e));
    e->buf = buf;
    e->cap = cap;
    e->channels = channels;
    for (unsigned i = 0; i < GYL_MAX_CH; i++) {
        e->mean16[i] = GYL_MEAN_INIT16;
    }
}

bool gyl_enc_put(gyl_enc_t *e, const int16_t *v)
{
    if (e->count == 0) {
        if (e->cap - e->pos < (size_t)e->channels * 2u + GYL_MAX_SAMPLE_BYTES) {
            return false;
        }
        for (unsigned c = 0; c < e->channels; c++) {
            e->buf[e->pos++] = (uint8_t)((uint16_t)v[c] & 0xFFu);
            e->buf[e->pos++] = (uint8_t)((uint16_t)v[c] >> 8);
            e->prev[c] = v[c];
            e->prev2[c] = v[c];
        }
        e->count = 1;
        return true;
    }
    if (e->cap - e->pos < GYL_MAX_SAMPLE_BYTES) {
        return false;
    }
    for (unsigned c = 0; c < e->channels; c++) {
        int32_t pred = gyl_predict(e->prev[c], e->prev2[c], e->err1[c], e->err2[c]);
        uint32_t u = zigzag((int32_t)v[c] - pred);
        uint32_t k = gyl_rice_k(e->mean16[c]);
        uint32_t q = u >> k;
        if (q >= GYL_ESC_Q) {
            put_bits(e, 0xFFFFu, 16);
            put_bits(e, 0xFu, GYL_ESC_Q - 16u);
            put_bits(e, u, GYL_ESC_BITS);
        } else {
            while (q >= 16u) {
                put_bits(e, 0xFFFFu, 16);
                q -= 16u;
            }
            /* q ones followed by a zero == ((1 << q) - 1) << 1 in q + 1 bits */
            put_bits(e, ((1u << q) - 1u) << 1, q + 1u);
            if (k) {
                put_bits(e, u, k);
            }
        }
        GYL_ADAPT(e, c, v[c], u);
    }
    e->count++;
    return true;
}

size_t gyl_enc_finish(gyl_enc_t *e)
{
    if (e->nbits) {
        e->buf[e->pos++] = (uint8_t)(e->acc << (8u - e->nbits));
        e->nbits = 0;
        e->acc = 0;
    }
    return e->pos;
}
