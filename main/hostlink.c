#include "hostlink.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "app_config.h"
#ifdef CONFIG_GYROLOG_SELFTEST
#include "driver/uart.h"   /* QEMU has no USB Serial/JTAG: the protocol runs over the console UART */
#else
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "gyl_format.h"
#include "logstore.h"
#include "pm1.h"
#include "recorder.h"

#define SESSION_HOLD_US (60LL * 1000000LL)

static int64_t s_last_cmd = -SESSION_HOLD_US;
static char s_line[80];
static size_t s_len;
static uint8_t s_io[GYL_PAGE_SIZE];

/* ---- transport: USB Serial/JTAG (production) or UART0 (QEMU self-test build) ---- */
#ifdef CONFIG_GYROLOG_SELFTEST
void hostlink_init(void)
{
    uart_driver_install(UART_NUM_0, 512, 2048, 0, NULL, 0);
}

static int link_read_byte(uint8_t *c)
{
    return uart_read_bytes(UART_NUM_0, c, 1, 0);
}

static int link_write_some(const uint8_t *p, size_t n)
{
    return uart_write_bytes(UART_NUM_0, p, n);
}
#else
void hostlink_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 2048;
    cfg.rx_buffer_size = 256;
    usb_serial_jtag_driver_install(&cfg);
}

static int link_read_byte(uint8_t *c)
{
    return usb_serial_jtag_read_bytes(c, 1, 0);
}

static int link_write_some(const uint8_t *p, size_t n)
{
    return usb_serial_jtag_write_bytes(p, n, pdMS_TO_TICKS(250));
}
#endif

bool hostlink_session_active(int64_t now_us)
{
    return now_us - s_last_cmd < SESSION_HOLD_US;
}

static bool link_write(const void *buf, size_t n)
{
    const uint8_t *p = buf;
    int stalls = 0;
    while (n) {
        int w = link_write_some(p, n);
        if (w <= 0) {
            if (++stalls >= 8) {
                return false; /* host went away */
            }
            continue;
        }
        stalls = 0;
        p += w;
        n -= (size_t)w;
    }
    return true;
}

static void reply(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n > 0) {
        link_write(b, (size_t)n < sizeof(b) ? (size_t)n : sizeof(b) - 1);
    }
}

static void do_hello(void)
{
    esp_log_level_set("*", ESP_LOG_NONE); /* keep the binary stream clean */
    const logstore_info_t *li = logstore_info();
    time_t now = time(NULL);
    reply("GYLOG 1 fw=%s pages=%u used=%u batt=%u rec=%d unix=%u odr=%d ch=%d orient=%s\n", FW_VERSION,
          (unsigned)li->total_pages, (unsigned)li->next_page, (unsigned)pm1_battery_mv(), rec_status()->active ? 1 : 0,
          (unsigned)(now > 0 ? now : 0), GYROLOG_ODR_HZ, GYROLOG_CHANNELS, GYROLOG_ORIENTATION);
}

static void do_read(const char *args)
{
    char *end;
    unsigned long first = strtoul(args, &end, 10);
    unsigned long count = strtoul(end, NULL, 10);
    const logstore_info_t *li = logstore_info();
    if (rec_status()->active) {
        reply("ERR recording\n");
        return;
    }
    if (count == 0 || count > 256 || first + count > li->total_pages) {
        reply("ERR range\n");
        return;
    }
    reply("OK %lu %lu\n", first, count);
    for (unsigned long i = 0; i < count; i++) {
        if (logstore_read((uint32_t)(first + i), 1, s_io) != ESP_OK) {
            memset(s_io, 0, sizeof(s_io)); /* the host will see a corrupt page */
        }
        if (!link_write(s_io, sizeof(s_io))) {
            return;
        }
    }
}

static void do_erase(void)
{
    if (rec_status()->active) {
        reply("ERR recording\n");
        return;
    }
    reply("%s", logstore_erase_all() == ESP_OK ? "OK\n" : "ERR flash\n");
}

static void do_time(const char *args)
{
    long long t = atoll(args);
    if (t < 1600000000LL) {
        reply("ERR time\n");
        return;
    }
    struct timeval tv = {.tv_sec = (time_t)t, .tv_usec = 0};
    settimeofday(&tv, NULL);
    reply("OK\n");
}

static void handle_line(char *line)
{
    s_last_cmd = esp_timer_get_time();
    if (strncmp(line, "HELLO", 5) == 0) {
        do_hello();
    } else if (strncmp(line, "READ ", 5) == 0) {
        do_read(line + 5);
    } else if (strcmp(line, "ERASE") == 0) {
        do_erase();
    } else if (strncmp(line, "TIME ", 5) == 0) {
        do_time(line + 5);
    } else if (line[0]) {
        reply("ERR unknown\n");
    }
}

void hostlink_poll(void)
{
    uint8_t c;
    while (link_read_byte(&c) == 1) {
        if (c == '\n') {
            s_line[s_len] = '\0';
            handle_line(s_line);
            s_len = 0;
        } else if (c != '\r') {
            if (s_len < sizeof(s_line) - 1) {
                s_line[s_len++] = (char)c;
            } else {
                s_len = 0; /* overlong garbage line */
            }
        }
    }
}
