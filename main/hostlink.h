/*
 * USB serial protocol used by tools/gyrostick.py (over the chip's USB Serial/JTAG port).
 * Line based requests, binary page payloads:
 *   HELLO        -> "GYLOG 1 fw=.. pages=N used=W batt=mV rec=0|1 unix=T odr=.. ch=.. orient=.."
 *   READ a n     -> "OK a n\n" followed by n * 4096 raw bytes
 *   ERASE        -> "OK\n" once all used pages are erased
 *   TIME t       -> "OK\n"  (sets the system clock, unix seconds)
 * Anything else answers "ERR <reason>\n".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void hostlink_init(void);
/* Poll for requests; cheap when nothing arrives. */
void hostlink_poll(void);
/* A host spoke recently: stay awake. */
bool hostlink_session_active(int64_t now_us);
