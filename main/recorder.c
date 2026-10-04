#include "recorder.h"

#include <string.h>
#include <time.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gyl_codec.h"
#include "gyl_format.h"
#include "logstore.h"

static const char *TAG = "rec";

#define TICKS_PER_SAMPLE (GYL_ST_TICKS_PER_SEC / GYROLOG_ODR_HZ)
#define GAP_TOL_TICKS    (4 * TICKS_PER_SAMPLE)

static uint8_t s_page[GYL_PAGE_SIZE];
static gyl_enc_t s_enc;
static rec_status_t s_st;

static bool     s_page_open;
static uint32_t s_page_seq;
static uint32_t s_page_first_idx;
static bool     s_pending_gap;     /* next page starts after a discontinuity */
static bool     s_page_gap;        /* the open page carries the GAP flag */
static uint32_t s_idx;             /* samples recorded so far (index of the next sample) */
static uint32_t s_unix;

static bool     s_have_last_st;
static uint32_t s_last_st;
static bool     s_anchor_valid;
static uint32_t s_anchor_idx, s_anchor_st, s_anchor_us;
static uint16_t s_batt_mv;

static void open_page(void)
{
    gyl_enc_init(&s_enc, s_page + GYL_HDR_SIZE, GYL_PAYLOAD_MAX, GYROLOG_CHANNELS);
    s_page_first_idx = s_idx;
    s_page_gap = s_pending_gap;
    s_pending_gap = false;
    s_page_open = true;
}

static void seal_page(bool last)
{
    if (!s_page_open || s_enc.count == 0) {
        return;
    }
    size_t plen = gyl_enc_finish(&s_enc);
    memset(s_page + GYL_HDR_SIZE + plen, 0xFF, GYL_PAGE_SIZE - GYL_HDR_SIZE - plen);

    gyl_page_info_t inf;
    memset(&inf, 0, sizeof(inf));
    inf.flags = (uint8_t)((s_page_seq == 0 ? GYL_FLAG_FIRST : 0) | (last ? GYL_FLAG_LAST : 0) |
                          (s_page_gap ? GYL_FLAG_GAP : 0) | (LIGHT_SLEEP_REC ? GYL_FLAG_LSLEEP : 0));
    inf.payload_len = (uint16_t)plen;
    inf.session_id = s_st.session_id;
    inf.page_seq = s_page_seq;
    inf.first_idx = s_page_first_idx;
    inf.n_samples = s_enc.count;
    inf.odr_hz = GYROLOG_ODR_HZ;
    inf.gyro_fs_dps = GYROLOG_GYRO_DPS;
    inf.acc_fs_g = GYROLOG_ACC_G;
    inf.channels = GYROLOG_CHANNELS;
    inf.session_unix = s_unix;
    inf.sync_idx = s_anchor_valid ? s_anchor_idx : s_page_first_idx;
    inf.sync_st24 = s_anchor_st;
    inf.sync_us32 = s_anchor_us;
    inf.batt_mv = s_batt_mv;
    strncpy(inf.orient, GYROLOG_ORIENTATION, GYL_ORIENT_LEN);
    gyl_page_seal(s_page, &inf);

    esp_err_t err = logstore_append(s_page);
    if (err == ESP_ERR_NO_MEM) {
        s_st.full = true;
    } else if (err != ESP_OK) {
        s_st.error = true;
    } else {
        s_st.pages++;
        s_st.bytes += (uint32_t)plen;
    }
    s_page_seq++;
    s_page_open = false;
}

esp_err_t rec_begin(void)
{
    const logstore_info_t *li = logstore_info();
    if (li->next_page >= li->total_pages) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_st, 0, sizeof(s_st));
    s_st.active = true;
    s_st.session_id = li->next_session_id;
    s_page_open = false;
    s_page_seq = 0;
    s_pending_gap = false;
    s_idx = 0;
    s_have_last_st = false;
    s_anchor_valid = false;
    time_t now = time(NULL);
    s_unix = now > 1600000000 ? (uint32_t)now : 0;
    ESP_LOGI(TAG, "session %u started", (unsigned)s_st.session_id);
    return ESP_OK;
}

void rec_feed(const imu_sample_t *s, size_t n, const imu_burst_t *b, uint16_t batt_mv)
{
    if (!s_st.active || s_st.full || s_st.error) {
        return;
    }
    if (batt_mv) {
        s_batt_mv = batt_mv;
    }

    /* Discontinuity check: sensor time must advance by exactly the number of frames we saw. */
    if (b->has_st && s_have_last_st && n + b->dropped + b->skipped > 0) {
        uint32_t delta = (b->st24 - s_last_st) & 0xFFFFFFu;
        int32_t expected = (int32_t)((n + b->dropped) * TICKS_PER_SAMPLE);
        int32_t err = (int32_t)delta - expected;
        if (b->overflow || err > (int32_t)GAP_TOL_TICKS || err < -(int32_t)GAP_TOL_TICKS) {
            seal_page(false);
            s_pending_gap = true;
            s_st.gaps++;
        }
    }
    if (b->has_st) {
        s_last_st = b->st24;
        s_have_last_st = true;
    }

    for (size_t i = 0; i < n; i++) {
        if (!s_page_open) {
            open_page();
        }
        if (!gyl_enc_put(&s_enc, s[i].v)) {
            seal_page(false);
            if (s_st.full || s_st.error) {
                return;
            }
            open_page();
            gyl_enc_put(&s_enc, s[i].v);
        }
        s_idx++;
        s_st.samples++;
    }

    if (n && b->has_st) {
        s_anchor_valid = true;
        s_anchor_idx = s_idx - 1;
        s_anchor_st = b->st24;
        s_anchor_us = (uint32_t)esp_timer_get_time();
    }
}

esp_err_t rec_end(void)
{
    if (!s_st.active) {
        return ESP_ERR_INVALID_STATE;
    }
    seal_page(true);
    s_st.active = false;
    ESP_LOGI(TAG, "session %u closed: %u samples, %u pages, %u gaps", (unsigned)s_st.session_id,
             (unsigned)s_st.samples, (unsigned)s_st.pages, (unsigned)s_st.gaps);
    return (s_st.full || s_st.error) ? ESP_FAIL : ESP_OK;
}

const rec_status_t *rec_status(void)
{
    return &s_st;
}
