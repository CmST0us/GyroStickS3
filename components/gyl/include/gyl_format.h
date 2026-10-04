/*
 * GYL log page format
 *
 * The log partition is an append-only array of 4 KiB pages.  Every page is
 * self-contained: a 64 byte header followed by a bit-packed payload of
 * IMU samples.  Pages are written exactly once (erase + one program), so a
 * power loss can only ever damage the page that was being written, and the
 * damage is detected by the CRC.
 *
 * All multi-byte header fields are little endian.
 *
 *   off size field
 *     0    4  magic          "GYPL" (0x4C505947)
 *     4    1  version        GYL_VERSION
 *     5    1  flags          GYL_FLAG_*
 *     6    2  payload_len    valid payload bytes after the header
 *     8    4  session_id     monotonically increasing recording id
 *    12    4  page_seq       page number inside the session (0 based)
 *    16    4  first_idx      index of the first sample of this page in the session
 *    20    2  n_samples      samples stored in this page
 *    22    2  odr_hz         nominal IMU output data rate
 *    24    2  gyro_fs_dps    gyroscope full scale (+/- dps)
 *    26    1  acc_fs_g       accelerometer full scale (+/- g)
 *    27    1  channels       3 = gyro only, 6 = gyro + accel
 *    28    4  session_unix   unix time at session start (0 = clock not set)
 *    32    4  sync_idx       sample index the three sync values below refer to
 *    36    4  sync_st24      BMI270 sensor time (39.0625 us ticks, 24 bit) at sync_idx
 *    40    4  sync_us32      MCU esp_timer time (us, low 32 bit) at sync_idx
 *    44    2  batt_mv        battery voltage when the page was written
 *    46    8  orient         Gyroflow IMU orientation string, NUL padded
 *    54    6  reserved       0
 *    60    4  crc32          CRC-32 (IEEE) over bytes [0,60) and the payload
 *
 * Payload: the first sample is stored raw (channels x int16 LE).  Every
 * following sample is stored as, per channel, the zig-zag coded difference to
 * the previous sample of the same channel, Rice coded with an adaptive
 * parameter (see gyl_codec.h).
 */
#ifndef GYL_FORMAT_H
#define GYL_FORMAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GYL_PAGE_SIZE   4096u
#define GYL_HDR_SIZE    64u
#define GYL_PAYLOAD_MAX (GYL_PAGE_SIZE - GYL_HDR_SIZE)
#define GYL_MAGIC       0x4C505947u
#define GYL_VERSION     1u
#define GYL_ORIENT_LEN  8u

#define GYL_FLAG_FIRST  0x01u /* first page of a session */
#define GYL_FLAG_LAST   0x02u /* session was stopped cleanly after this page */
#define GYL_FLAG_GAP    0x04u /* samples were lost right before this page */
#define GYL_FLAG_LSLEEP 0x08u /* MCU light sleep was used, sync_us32 is not a precise clock */

/* BMI270 sensor time: 24 bit counter, 39.0625 us per tick. */
#define GYL_ST_BITS      24u
#define GYL_ST_TICK_US   39.0625
#define GYL_ST_TICKS_PER_SEC 25600u

typedef struct {
    uint8_t  version;
    uint8_t  flags;
    uint16_t payload_len;
    uint32_t session_id;
    uint32_t page_seq;
    uint32_t first_idx;
    uint16_t n_samples;
    uint16_t odr_hz;
    uint16_t gyro_fs_dps;
    uint8_t  acc_fs_g;
    uint8_t  channels;
    uint32_t session_unix;
    uint32_t sync_idx;
    uint32_t sync_st24;
    uint32_t sync_us32;
    uint16_t batt_mv;
    char     orient[GYL_ORIENT_LEN + 1];
} gyl_page_info_t;

typedef enum {
    GYL_PAGE_OK = 0,
    GYL_PAGE_ERASED,  /* header is all 0xFF: never written */
    GYL_PAGE_CORRUPT, /* partially written / bit rot / foreign data */
} gyl_page_status_t;

uint32_t gyl_crc32(uint32_t crc, const uint8_t *data, size_t len);

/* Serialise `info` into page[0..GYL_HDR_SIZE) and seal the page with its CRC.
 * The payload (info->payload_len bytes) must already be in page[GYL_HDR_SIZE..]. */
void gyl_page_seal(uint8_t *page, const gyl_page_info_t *info);

/* Parse and verify a page. `out` may be NULL. */
gyl_page_status_t gyl_page_parse(const uint8_t *page, gyl_page_info_t *out);

/* Cheap check used while scanning the partition: only looks at the first bytes. */
bool gyl_page_header_erased(const uint8_t *hdr16);

#ifdef __cplusplus
}
#endif

#endif /* GYL_FORMAT_H */
