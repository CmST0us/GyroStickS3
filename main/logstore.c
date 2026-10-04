#include "logstore.h"

#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"

static const char *TAG = "logstore";

/* partitions.csv: gyrolog, data, 0x40  ->  type DATA (0x01), custom subtype 0x40 */
#define PART_TYPE    ESP_PARTITION_TYPE_DATA
#define PART_SUBTYPE ((esp_partition_subtype_t)0x40)

static const esp_partition_t *s_part;
static logstore_info_t s_info;

esp_err_t logstore_init(void)
{
    if (!s_part) {
        s_part = esp_partition_find_first(PART_TYPE, PART_SUBTYPE, "gyrolog");
        if (!s_part) {
            ESP_LOGE(TAG, "partition 'gyrolog' not found");
            return ESP_ERR_NOT_FOUND;
        }
    }
    memset(&s_info, 0, sizeof(s_info));
    s_info.total_pages = s_part->size / GYL_PAGE_SIZE;

    uint32_t last_used = 0;
    bool any = false;
    uint32_t max_session = 0;
    uint32_t prev_session = 0xFFFFFFFFu;
    uint8_t hdr[16];
    for (uint32_t p = 0; p < s_info.total_pages; p++) {
        if (esp_partition_read(s_part, (size_t)p * GYL_PAGE_SIZE, hdr, sizeof(hdr)) != ESP_OK) {
            return ESP_FAIL;
        }
        if (gyl_page_header_erased(hdr)) {
            continue;
        }
        last_used = p;
        any = true;
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        if (magic == GYL_MAGIC) {
            uint32_t sid = (uint32_t)hdr[8] | ((uint32_t)hdr[9] << 8) | ((uint32_t)hdr[10] << 16) | ((uint32_t)hdr[11] << 24);
            if (sid > max_session) {
                max_session = sid;
            }
            if (sid != prev_session) {
                s_info.sessions++;
                prev_session = sid;
            }
        }
    }
    s_info.next_page = any ? last_used + 1 : 0;
    s_info.next_session_id = max_session + 1;
    ESP_LOGI(TAG, "%u/%u pages used, %u recording(s), next session %u", (unsigned)s_info.next_page,
             (unsigned)s_info.total_pages, (unsigned)s_info.sessions, (unsigned)s_info.next_session_id);
    return ESP_OK;
}

const logstore_info_t *logstore_info(void)
{
    return &s_info;
}

esp_err_t logstore_append(const uint8_t *page)
{
    if (!s_part) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_info.next_page >= s_info.total_pages) {
        return ESP_ERR_NO_MEM;
    }
    size_t off = (size_t)s_info.next_page * GYL_PAGE_SIZE;
    esp_err_t err = esp_partition_erase_range(s_part, off, GYL_PAGE_SIZE);
    if (err == ESP_OK) {
        err = esp_partition_write(s_part, off, page, GYL_PAGE_SIZE);
    }
    /* Whatever happened, never reuse this page: a failed write may have left it half programmed. */
    s_info.next_page++;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "page write failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t logstore_read(uint32_t first, uint32_t count, void *buf)
{
    if (!s_part) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((uint64_t)first + count > s_info.total_pages) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_partition_read(s_part, (size_t)first * GYL_PAGE_SIZE, buf, (size_t)count * GYL_PAGE_SIZE);
}

esp_err_t logstore_erase_all(void)
{
    if (!s_part) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_info.next_page == 0) {
        return ESP_OK;
    }
    /* erase in 64 KiB blocks where possible; round the used range up to whole sectors */
    size_t used = (size_t)s_info.next_page * GYL_PAGE_SIZE;
    size_t chunk = 64 * 1024;
    for (size_t off = 0; off < used; off += chunk) {
        size_t len = used - off < chunk ? used - off : chunk;
        len = (len + 4095) & ~(size_t)4095;
        esp_err_t err = esp_partition_erase_range(s_part, off, len);
        if (err != ESP_OK) {
            return err;
        }
    }
    return logstore_init();
}
