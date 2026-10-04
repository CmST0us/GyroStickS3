#include <stdio.h>
#include <string.h>

#include "btn_logic.h"

static int failures;
#define CHECK(c)                                                                      \
    do {                                                                              \
        if (!(c)) {                                                                   \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                       \
            failures++;                                                               \
        }                                                                             \
    } while (0)

#define LONG_US 1200000

/* Feed a level sequence sampled every 10 ms; returns the events fired (c = click, L = long). */
static void run(btn_state_t *b, const char *levels, int64_t *t, char *events)
{
    for (const char *p = levels; *p; p++) {
        btn_event_t e = btn_step(b, *p == '1', *t, LONG_US);
        if (e == BTN_EV_CLICK) *events++ = 'c';
        if (e == BTN_EV_LONG) *events++ = 'L';
        *t += 10000;
    }
    *events = 0;
}

static void held(char *buf, int ms)
{
    memset(buf, '1', ms / 10);
    buf[ms / 10] = 0;
}

int main(void)
{
    char ev[16], lv[400];
    int64_t t = 1000000;
    btn_state_t b;

    /* a normal click: 150 ms down, then idle */
    memset(&b, 0, sizeof(b));
    held(lv, 150); strcat(lv, "0000000000");
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "c") == 0);

    /* contact bounce on press and release is filtered: one click */
    memset(&b, 0, sizeof(b));
    run(&b, "0101011111111111110100000000", &t, ev);
    CHECK(strcmp(ev, "c") == 0);

    /* glitch shorter than the debounce time: nothing */
    memset(&b, 0, sizeof(b));
    run(&b, "000110000000000000", &t, ev);
    CHECK(strcmp(ev, "") == 0);

    /* long press fires once while held, release afterwards is not a click */
    memset(&b, 0, sizeof(b));
    held(lv, 1600); strcat(lv, "00000000");
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "L") == 0);

    /* just below the long press time is still a click */
    memset(&b, 0, sizeof(b));
    held(lv, 1100); strcat(lv, "00000000");
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "c") == 0);

    /* two clicks in a row */
    memset(&b, 0, sizeof(b));
    held(lv, 100); strcat(lv, "0000000000");
    char lv2[200]; held(lv2, 100); strcat(lv, lv2); strcat(lv, "0000000000");
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "cc") == 0);

    /* press that woke the device: swallowed, but the next click works */
    memset(&b, 0, sizeof(b));
    btn_swallow(&b, true);
    held(lv, 800); strcat(lv, "0000000000");
    strcat(lv, "1111111111"); strcat(lv, "0000000000"); /* a real click afterwards */
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "c") == 0);

    /* swallow with the button already released must not eat the next click */
    memset(&b, 0, sizeof(b));
    btn_swallow(&b, false);
    run(&b, "11111111110000000000", &t, ev);
    CHECK(strcmp(ev, "c") == 0);

    /* a long press that is swallowed never fires LONG */
    memset(&b, 0, sizeof(b));
    btn_swallow(&b, true);
    held(lv, 2000); strcat(lv, "0000000000");
    run(&b, lv, &t, ev);
    CHECK(strcmp(ev, "") == 0);

    /* busy tracking */
    memset(&b, 0, sizeof(b));
    CHECK(!btn_busy(&b, t));
    btn_step(&b, true, t, LONG_US);
    CHECK(btn_busy(&b, t + 1000));

    if (failures) {
        printf("%d FAILURES\n", failures);
        return 1;
    }
    printf("buttons: ALL OK\n");
    return 0;
}
