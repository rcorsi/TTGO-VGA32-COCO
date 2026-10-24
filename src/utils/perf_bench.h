/*
 * ============================================================
 *        CoCo 2&3 Emulator for ESP32-TTGO-VGA32-COCO
 *   (C) 2026 Reinaldo Torres / CoCo Byte Club
 *   https://github.com/reyco2000/TTGO-VGA32-COCO
 *   Based on XRoar , co-developed with Claude Code
 *   GPL-3.0-or-later License
 * ============================================================
 *  File   : perf_bench.h
 *  Module : Boot-time CPU interpreter micro-benchmark (-DPERF_BENCH=1)
 * ============================================================
*/

#ifndef PERF_BENCH_H
#define PERF_BENCH_H

#include "../../config.h"

#ifndef PERF_BENCH                  // override: -DPERF_BENCH=1
#define PERF_BENCH 0
#endif

#if PERF_BENCH
void perf_bench_run(void);          // prints a table on Serial; call once at boot
#else
static inline void perf_bench_run(void) {}
#endif

#endif // PERF_BENCH_H
