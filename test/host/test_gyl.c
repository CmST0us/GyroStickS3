/* Host test: gyl codec round trip, page sealing, page parsing, compression ratio.
 * Also writes test_pages.bin + test_samples.csv for the python cross check. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gyl_codec.h"
#include "gyl_format.h"

static int failures;
#define CHECK(c)                                                                      \
    do {                                                                              \
        if (!(c)) {                                                                   \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                       \
            failures++;                                                               \
        }                                                                             \
    } while (0)

static uint32_t rng = 12345;
static uint32_t rnd(void)
{
    rng = rng * 1664525u + 1013904223u;
    return rng >> 8;
}
static double gauss(void)
{
    double u1 = (rnd() + 1.0) / 16777217.0, u2 = (rnd() + 1.0) / 16777217.0;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}
static int16_t clamp16(double v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)lrint(v);
}

/* Synthetic hand-held camera: slow pans + shake, gravity on accel, sensor noise. */
static void synth(int16_t (*s)[6], size_t n, double odr, double motion_dps, double noise_lsb)
{
    for (size_t i = 0; i < n; i++) {
        double t = i / odr;
        double w[3], a[3];
        for (int c = 0; c < 3; c++) {
            double f1 = 0.4 + 0.3 * c, f2 = 2.1 + 0.9 * c, f3 = 7.0 + 1.3 * c;
            w[c] = motion_dps * (0.6 * sin(6.283 * f1 * t + c) + 0.3 * sin(6.283 * f2 * t) + 0.1 * sin(6.283 * f3 * t));
        }
        a[0] = 0.1 * sin(6.283 * 1.3 * t);
        a[1] = -0.05 * sin(6.283 * 0.7 * t);
        a[2] = 1.0 + 0.08 * sin(6.283 * 2.9 * t);
        for (int c = 0; c < 3; c++) {
            s[i][c] = clamp16(w[c] * 32.8 + 15 + gauss() * noise_lsb);
            s[i][3 + c] = clamp16(a[c] * 4096.0 + gauss() * noise_lsb * 2);
        }
    }
}

static size_t encode_all(const int16_t (*s)[6], size_t n, unsigned ch, FILE *pages_out, uint32_t *npages)
{
    uint8_t page[GYL_PAGE_SIZE];
    gyl_enc_t e;
    size_t i = 0, total_payload = 0;
    uint32_t seq = 0;
    while (i < n) {
        memset(page, 0xFF, sizeof(page));
        gyl_enc_init(&e, page + GYL_HDR_SIZE, GYL_PAYLOAD_MAX, (uint8_t)ch);
        size_t first = i;
        while (i < n && gyl_enc_put(&e, s[i])) {
            i++;
        }
        size_t plen = gyl_enc_finish(&e);
        gyl_page_info_t inf;
        memset(&inf, 0, sizeof(inf));
        inf.flags = seq == 0 ? GYL_FLAG_FIRST : 0;
        inf.payload_len = (uint16_t)plen;
        inf.session_id = 7;
        inf.page_seq = seq;
        inf.first_idx = (uint32_t)first;
        inf.n_samples = e.count;
        inf.odr_hz = 200;
        inf.gyro_fs_dps = 1000;
        inf.acc_fs_g = 8;
        inf.channels = (uint8_t)ch;
        inf.session_unix = 1700000000u;
        inf.sync_idx = (uint32_t)(i - 1);
        inf.sync_st24 = (uint32_t)((i - 1) * 128u) & 0xFFFFFFu;
        inf.sync_us32 = (uint32_t)((i - 1) * 5000u);
        inf.batt_mv = 4000;
        strncpy(inf.orient, "Zxy", GYL_ORIENT_LEN);
        gyl_page_seal(page, &inf);
        if (pages_out) fwrite(page, 1, sizeof(page), pages_out);
        total_payload += plen;
        seq++;
        /* verify the page decodes to exactly the encoded samples */
        gyl_page_info_t chk;
        CHECK(gyl_page_parse(page, &chk) == GYL_PAGE_OK);
        CHECK(chk.n_samples == e.count && chk.first_idx == first);
        gyl_dec_t d;
        gyl_dec_init(&d, page + GYL_HDR_SIZE, chk.payload_len, (uint8_t)ch);
        int16_t v[6];
        for (size_t j = 0; j < chk.n_samples; j++) {
            CHECK(gyl_dec_get(&d, v));
            for (unsigned c = 0; c < ch; c++) {
                if (v[c] != s[first + j][c]) {
                    printf("mismatch page %u sample %zu ch %u: %d != %d\n", seq - 1, j, c, v[c], s[first + j][c]);
                    failures++;
                    return total_payload;
                }
            }
        }
    }
    if (npages) *npages = seq;
    return total_payload;
}

static void test_synthetic(const char *name, double odr, double motion, double noise, unsigned ch)
{
    size_t n = 60 * (size_t)odr; /* one minute */
    int16_t(*s)[6] = malloc(n * sizeof(*s));
    synth(s, n, odr, motion, noise);
    uint32_t np = 0;
    size_t bytes = encode_all((const int16_t(*)[6])s, n, ch, NULL, &np);
    printf("%-34s ch=%u  %6.2f bytes/sample  (%5.1f bits/axis)  %7.1f B/s  pages/min=%u\n", name, ch,
           (double)bytes / n, 8.0 * bytes / n / ch, (double)bytes / 60.0, np);
    free(s);
}

static void test_extremes(void)
{
    /* full scale random data: every residual escapes */
    size_t n = 3000;
    int16_t(*s)[6] = malloc(n * sizeof(*s));
    for (size_t i = 0; i < n; i++)
        for (int c = 0; c < 6; c++) s[i][c] = (int16_t)(rnd() & 0xFFFF);
    uint32_t np;
    encode_all((const int16_t(*)[6])s, n, 6, NULL, &np);
    /* alternating +/-32767 / -32768: max residuals */
    for (size_t i = 0; i < n; i++)
        for (int c = 0; c < 6; c++) s[i][c] = (i & 1) ? 32767 : -32768;
    encode_all((const int16_t(*)[6])s, n, 6, NULL, &np);
    /* constant */
    for (size_t i = 0; i < n; i++)
        for (int c = 0; c < 6; c++) s[i][c] = 0;
    encode_all((const int16_t(*)[6])s, n, 3, NULL, &np);
    free(s);
}

static void test_page_status(void)
{
    uint8_t page[GYL_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    CHECK(gyl_page_parse(page, NULL) == GYL_PAGE_ERASED);
    gyl_enc_t e;
    gyl_enc_init(&e, page + GYL_HDR_SIZE, GYL_PAYLOAD_MAX, 6);
    int16_t v[6] = {1, 2, 3, 4, 5, 6};
    CHECK(gyl_enc_put(&e, v));
    CHECK(gyl_enc_put(&e, v));
    gyl_page_info_t inf;
    memset(&inf, 0, sizeof(inf));
    inf.payload_len = (uint16_t)gyl_enc_finish(&e);
    inf.n_samples = e.count;
    inf.channels = 6;
    gyl_page_seal(page, &inf);
    CHECK(gyl_page_parse(page, NULL) == GYL_PAGE_OK);
    page[GYL_HDR_SIZE + 3] ^= 0x10; /* flip a payload bit */
    CHECK(gyl_page_parse(page, NULL) == GYL_PAGE_CORRUPT);
    page[GYL_HDR_SIZE + 3] ^= 0x10;
    page[9] ^= 1; /* flip a header bit */
    CHECK(gyl_page_parse(page, NULL) == GYL_PAGE_CORRUPT);
    /* a torn write: header programmed, rest still erased */
    memset(page + 200, 0xFF, sizeof(page) - 200);
    CHECK(gyl_page_parse(page, NULL) == GYL_PAGE_CORRUPT);
    /* known CRC-32 vector */
    CHECK(gyl_crc32(0, (const uint8_t *)"123456789", 9) == 0xCBF43926u);
}

int main(void)
{
    test_page_status();
    test_extremes();
    printf("--- compression on synthetic hand-held motion (60 s) ---\n");
    test_synthetic("200Hz still on tripod (noise 2)", 200, 0.5, 2, 6);
    test_synthetic("200Hz handheld walking 60dps", 200, 60, 2, 6);
    test_synthetic("200Hz handheld 60dps gyro only", 200, 60, 2, 3);
    test_synthetic("200Hz vigorous 300dps", 200, 300, 3, 6);
    test_synthetic("400Hz handheld 60dps", 400, 60, 2, 6);
    test_synthetic("100Hz handheld 60dps", 100, 60, 2, 6);

    /* artefacts for the python cross check */
    size_t n = 2500;
    int16_t(*s)[6] = malloc(n * sizeof(*s));
    synth(s, n, 200, 120, 2.5);
    s[100][0] = 32767; s[101][0] = -32768; s[102][1] = 32767; s[103][1] = -32768; /* force escapes */
    FILE *fp = fopen("test_pages.bin", "wb");
    FILE *fc = fopen("test_samples.csv", "w");
    uint32_t np;
    encode_all((const int16_t(*)[6])s, n, 6, fp, &np);
    for (size_t i = 0; i < n; i++)
        fprintf(fc, "%d,%d,%d,%d,%d,%d\n", s[i][0], s[i][1], s[i][2], s[i][3], s[i][4], s[i][5]);
    fclose(fp);
    fclose(fc);
    free(s);
    printf("pages written for cross check: %u\n", np);

    if (failures) {
        printf("%d FAILURES\n", failures);
        return 1;
    }
    printf("ALL OK\n");
    return 0;
}
