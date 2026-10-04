#include "gyl_format.h"

#include <string.h>

static const uint32_t crc_nib[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

/* Standard CRC-32 (zlib compatible), nibble table version. Pass 0 as the initial crc. */
uint32_t gyl_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ crc_nib[crc & 0x0F];
        crc = (crc >> 4) ^ crc_nib[crc & 0x0F];
    }
    return ~crc;
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void gyl_page_seal(uint8_t *page, const gyl_page_info_t *info)
{
    uint8_t *h = page;
    memset(h, 0, GYL_HDR_SIZE);
    put32(h + 0, GYL_MAGIC);
    h[4] = GYL_VERSION;
    h[5] = info->flags;
    put16(h + 6, info->payload_len);
    put32(h + 8, info->session_id);
    put32(h + 12, info->page_seq);
    put32(h + 16, info->first_idx);
    put16(h + 20, info->n_samples);
    put16(h + 22, info->odr_hz);
    put16(h + 24, info->gyro_fs_dps);
    h[26] = info->acc_fs_g;
    h[27] = info->channels;
    put32(h + 28, info->session_unix);
    put32(h + 32, info->sync_idx);
    put32(h + 36, info->sync_st24);
    put32(h + 40, info->sync_us32);
    put16(h + 44, info->batt_mv);
    memcpy(h + 46, info->orient, GYL_ORIENT_LEN);
    uint32_t crc = gyl_crc32(0, h, 60);
    crc = gyl_crc32(crc, page + GYL_HDR_SIZE, info->payload_len);
    put32(h + 60, crc);
}

bool gyl_page_header_erased(const uint8_t *hdr16)
{
    for (int i = 0; i < 16; i++) {
        if (hdr16[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

gyl_page_status_t gyl_page_parse(const uint8_t *page, gyl_page_info_t *out)
{
    if (gyl_page_header_erased(page)) {
        return GYL_PAGE_ERASED;
    }
    if (get32(page) != GYL_MAGIC || page[4] != GYL_VERSION) {
        return GYL_PAGE_CORRUPT;
    }
    uint16_t plen = get16(page + 6);
    if (plen > GYL_PAYLOAD_MAX) {
        return GYL_PAGE_CORRUPT;
    }
    uint32_t crc = gyl_crc32(0, page, 60);
    crc = gyl_crc32(crc, page + GYL_HDR_SIZE, plen);
    if (crc != get32(page + 60)) {
        return GYL_PAGE_CORRUPT;
    }
    if (out) {
        memset(out, 0, sizeof(*out));
        out->version = page[4];
        out->flags = page[5];
        out->payload_len = plen;
        out->session_id = get32(page + 8);
        out->page_seq = get32(page + 12);
        out->first_idx = get32(page + 16);
        out->n_samples = get16(page + 20);
        out->odr_hz = get16(page + 22);
        out->gyro_fs_dps = get16(page + 24);
        out->acc_fs_g = page[26];
        out->channels = page[27];
        out->session_unix = get32(page + 28);
        out->sync_idx = get32(page + 32);
        out->sync_st24 = get32(page + 36);
        out->sync_us32 = get32(page + 40);
        out->batt_mv = get16(page + 44);
        memcpy(out->orient, page + 46, GYL_ORIENT_LEN);
        out->orient[GYL_ORIENT_LEN] = '\0';
    }
    return GYL_PAGE_OK;
}
