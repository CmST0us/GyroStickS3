/*
 * Append-only page store on the "gyrolog" data partition.
 * Every 4 KiB page is erased and programmed exactly once (see gyl_format.h).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "gyl_format.h"

typedef struct {
    uint32_t total_pages;
    uint32_t next_page;        /* first never-used page, == total_pages when full */
    uint32_t next_session_id;
    uint32_t sessions;         /* distinct recordings found */
} logstore_info_t;

/* Locate the partition and scan it. Safe to call again to refresh. */
esp_err_t logstore_init(void);
const logstore_info_t *logstore_info(void);

/* Erase the next free page and program `page` (GYL_PAGE_SIZE bytes) into it. */
esp_err_t logstore_append(const uint8_t *page);
/* Read `count` pages starting at `first`. */
esp_err_t logstore_read(uint32_t first, uint32_t count, void *buf);
/* Erase every page in use. */
esp_err_t logstore_erase_all(void);
