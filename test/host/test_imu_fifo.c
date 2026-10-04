#include <stdio.h>
#include <string.h>

#include "imu_fifo.h"

static int failures;
#define CHECK(c)                                                                      \
    do {                                                                              \
        if (!(c)) {                                                                   \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                       \
            failures++;                                                               \
        }                                                                             \
    } while (0)

static size_t put16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((uint16_t)v >> 8);
    return 2;
}

static size_t gyr_acc_frame(uint8_t *p, int base)
{
    size_t n = 0;
    p[n++] = 0x8C;
    for (int i = 0; i < 6; i++) n += put16(p + n, (int16_t)(base + i));
    return n;
}

int main(void)
{
    uint8_t buf[512];
    imu_sample_t out[32];
    imu_fifo_info_t info;
    size_t used, n, pos;

    /* three gyro+acc frames followed by the sensor time frame */
    pos = 0;
    pos += gyr_acc_frame(buf + pos, 100);
    pos += gyr_acc_frame(buf + pos, 200);
    pos += gyr_acc_frame(buf + pos, -300);
    buf[pos++] = 0x44; buf[pos++] = 0x56; buf[pos++] = 0x34; buf[pos++] = 0x12;
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 32, &info, &used);
    CHECK(n == 3 && used == pos);
    CHECK(out[0].v[0] == 100 && out[0].v[3] == 103 && out[0].v[5] == 105);
    CHECK(out[2].v[0] == -300 && out[2].v[2] == -298);
    CHECK(info.has_st && info.st24 == 0x123456u);

    /* gyro only frames in 3 channel mode, acc zeroed */
    pos = 0;
    buf[pos++] = 0x88;
    for (int i = 0; i < 3; i++) pos += put16(buf + pos, (int16_t)(i - 1));
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, false, out, 32, &info, &used);
    CHECK(n == 1 && out[0].v[0] == -1 && out[0].v[2] == 1 && out[0].v[3] == 0);

    /* acc-only frame while gyro is required: dropped, not emitted */
    pos = 0;
    buf[pos++] = 0x84;
    for (int i = 0; i < 3; i++) pos += put16(buf + pos, 7);
    pos += gyr_acc_frame(buf + pos, 5);
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 32, &info, &used);
    CHECK(n == 1 && out[0].v[0] == 5 && used == pos);

    /* gyro-only frame in 6 channel mode: dropped (counted) */
    pos = 0;
    buf[pos++] = 0x88;
    for (int i = 0; i < 3; i++) pos += put16(buf + pos, 7);
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 32, &info, &used);
    CHECK(n == 0 && info.dropped == 1);

    /* skip frame + input config frame + tag bits in the header */
    pos = 0;
    buf[pos++] = 0x40; buf[pos++] = 3;
    buf[pos++] = 0x48; buf[pos++] = 1; buf[pos++] = 2; buf[pos++] = 3; buf[pos++] = 4;
    pos += gyr_acc_frame(buf + pos, 10);
    buf[pos - 13] |= 0x03; /* interrupt tag bits */
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 32, &info, &used);
    CHECK(n == 1 && info.skipped == 3 && out[0].v[0] == 10);

    /* partial frame at the end is not consumed */
    pos = 0;
    pos += gyr_acc_frame(buf + pos, 1);
    size_t full = pos;
    pos += gyr_acc_frame(buf + pos, 2);
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos - 5, true, out, 32, &info, &used);
    CHECK(n == 1 && used == full);
    /* Real BMI270 behaviour (seen on hardware): a frame cut off by the end of a read is re-sent from its header on
     * the next read, so the caller drops the leftover and parses the next read on its own. Frames 1, 2, 3 must come
     * out exactly once and in order. */
    {
        uint8_t second[64];
        size_t slen = 0;
        slen += gyr_acc_frame(second + slen, 2);   /* the cut-off frame, complete this time */
        slen += gyr_acc_frame(second + slen, 3);
        imu_sample_t all[4];
        size_t total = 0;
        memset(&info, 0, sizeof(info));
        total += imu_fifo_parse(buf, pos - 5, true, all, 4, &info, &used);          /* read 1: frame 1 + 8 stray bytes */
        total += imu_fifo_parse(second, slen, true, all + total, 4 - total, &info, &used); /* read 2 */
        CHECK(total == 3 && all[0].v[0] == 1 && all[1].v[0] == 2 && all[2].v[0] == 3);
        CHECK(!info.invalid);
    }

    /* over-read marker ends parsing */
    pos = 0;
    pos += gyr_acc_frame(buf + pos, 1);
    buf[pos++] = 0x80; buf[pos++] = 0x00; buf[pos++] = 0x00;
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 32, &info, &used);
    CHECK(n == 1 && used == pos);

    /* garbage header */
    buf[0] = 0x13;
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, 10, true, out, 32, &info, &used);
    CHECK(n == 0 && info.invalid);

    /* output buffer limit: resume where we stopped */
    pos = 0;
    for (int i = 0; i < 5; i++) pos += gyr_acc_frame(buf + pos, i * 10);
    memset(&info, 0, sizeof(info));
    n = imu_fifo_parse(buf, pos, true, out, 2, &info, &used);
    CHECK(n == 2 && used == 2 * 13);
    n = imu_fifo_parse(buf + used, pos - used, true, out, 32, &info, &used);
    CHECK(n == 3 && out[0].v[0] == 20);

    if (failures) {
        printf("%d FAILURES\n", failures);
        return 1;
    }
    printf("imu_fifo: ALL OK\n");
    return 0;
}
