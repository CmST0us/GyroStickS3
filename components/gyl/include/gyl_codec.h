/*
 * GYL sample codec: delta + adaptive Rice coding of int16 IMU samples.
 *
 * Per channel the encoder keeps
 *     prev, prev2 : the previous two sample values
 *     err1, err2  : 16 x EMA (alpha = 1/16) of |x - prev| and |x - (2*prev - prev2)|
 *     mean16      : 16 x EMA (alpha = 1/16) of the zig-zag residual
 * It predicts with  pred = prev  or, when the second-order predictor has had the
 * smaller recent error (err2 < err1),  pred = clamp16(2*prev - prev2), and codes
 *     u = zigzag(x - pred)
 * with Rice parameter
 *     k = floor(log2(mean16/16 + 1))
 * Quotient q = u >> k is sent in unary (q ones, one zero) followed by the k low bits.
 * q >= GYL_ESC_Q is escaped as GYL_ESC_Q ones followed by 17 raw bits of u.
 *
 * Bits are packed MSB first.  The first sample of a page is stored raw so every
 * page can be decoded on its own.
 *
 * The decoder lives in gyl_decode.c and is only built for host tools/tests; the
 * firmware never needs to decode.
 */
#ifndef GYL_CODEC_H
#define GYL_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GYL_MAX_CH      6u
#define GYL_ESC_Q       20u
#define GYL_ESC_BITS    17u
#define GYL_MEAN_INIT16 (8u * 16u)

/* Worst case size of one coded sample in bytes (6 ch x (20 + 17) bits, rounded up). */
#define GYL_MAX_SAMPLE_BYTES 30u

typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   pos;
    uint32_t acc;
    uint32_t nbits;
    uint8_t  channels;
    uint16_t count;
    int16_t  prev[GYL_MAX_CH];
    int16_t  prev2[GYL_MAX_CH];
    uint32_t err1[GYL_MAX_CH];
    uint32_t err2[GYL_MAX_CH];
    uint32_t mean16[GYL_MAX_CH];
} gyl_enc_t;

void   gyl_enc_init(gyl_enc_t *e, uint8_t *buf, size_t cap, uint8_t channels);
/* Returns false (and writes nothing) when the page cannot take another sample. */
bool   gyl_enc_put(gyl_enc_t *e, const int16_t *v);
/* Flush pending bits; returns the number of payload bytes produced. */
size_t gyl_enc_finish(gyl_enc_t *e);

typedef struct {
    const uint8_t *buf;
    size_t   len;
    size_t   pos;
    uint32_t acc;
    uint32_t nbits;
    uint8_t  channels;
    uint16_t count;
    int16_t  prev[GYL_MAX_CH];
    int16_t  prev2[GYL_MAX_CH];
    uint32_t err1[GYL_MAX_CH];
    uint32_t err2[GYL_MAX_CH];
    uint32_t mean16[GYL_MAX_CH];
} gyl_dec_t;

void gyl_dec_init(gyl_dec_t *d, const uint8_t *buf, size_t len, uint8_t channels);
/* Returns false when the stream is exhausted or malformed. */
bool gyl_dec_get(gyl_dec_t *d, int16_t *v);


/* ---- shared by encoder and decoder: keep these two in lock step ---- */

static inline uint32_t gyl_rice_k(uint32_t mean16)
{
    uint32_t m = (mean16 >> 4) + 1u;
    uint32_t k = 0;
    while ((m >> (k + 1u)) != 0u) {
        k++;
    }
    return k;
}

static inline int32_t gyl_predict(int16_t prev, int16_t prev2, uint32_t err1, uint32_t err2)
{
    if (err2 < err1) {
        int32_t p = 2 * (int32_t)prev - (int32_t)prev2;
        return p > 32767 ? 32767 : (p < -32768 ? -32768 : p);
    }
    return prev;
}

static inline uint32_t gyl_abs32(int32_t v)
{
    return v < 0 ? (uint32_t)(-v) : (uint32_t)v;
}

/* Update the per-channel adaptive state after sample x was coded with residual code u. */
#define GYL_ADAPT(st, c, x, u)                                                              \
    do {                                                                                    \
        (st)->mean16[c] = (st)->mean16[c] + (u) - ((st)->mean16[c] >> 4);                   \
        (st)->err1[c] += gyl_abs32((int32_t)(x) - (int32_t)(st)->prev[c]) - ((st)->err1[c] >> 4); \
        (st)->err2[c] += gyl_abs32((int32_t)(x) - (2 * (int32_t)(st)->prev[c] - (int32_t)(st)->prev2[c])) - ((st)->err2[c] >> 4); \
        (st)->prev2[c] = (st)->prev[c];                                                     \
        (st)->prev[c] = (int16_t)(x);                                                       \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* GYL_CODEC_H */
