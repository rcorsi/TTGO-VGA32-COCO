/*
 * ============================================================
 *        CoCo 2&3 Emulator for ESP32-TTGO-VGA32-COCO
 *   (C) 2026 Reinaldo Torres / CoCo Byte Club
 *   https://github.com/reyco2000/TTGO-VGA32-COCO
 *   Based on XRoar , co-developed with Claude Code
 *   GPL-3.0-or-later License
 * ============================================================
 *  File   : perf_bench.cpp
 *  Module : Boot-time CPU interpreter micro-benchmark (-DPERF_BENCH=1)
 * ============================================================
*/

// rick-performance.md Phase B2: host cycles per emulated instruction for a
// few fixed loops, with the emulated RAM in internal DRAM, in PSRAM, and in
// PSRAM with renderer-like PSRAM reads between scanline-sized run calls.

#include "../../config.h"

#if defined(ARDUINO) && PERF_BENCH

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <string.h>
#include "../core/mc6809.h"
#include "perf_bench.h"

static uint8_t* bench_pages[8];
static uint8_t bench_rd(uint16_t a) { return bench_pages[a >> 13][a & 0x1FFF]; }
static void    bench_wr(uint16_t a, uint8_t v) { bench_pages[a >> 13][a & 0x1FFF] = v; }

static inline uint32_t ccount(void) {
    uint32_t c;
    __asm__ __volatile__("rsr %0, ccount" : "=a"(c));
    return c;
}

struct BenchProg {
    const char* name;
    uint8_t code[8];
    int len;
    int insns;      // per pass
    int cycles;     // per pass, MC6809 timing
};

static const BenchProg progs[] = {
    { "LEAX -1,X / BNE",        { 0x30, 0x1F, 0x26, 0xFC },             4, 2,  8 },
    { "LDA <8 / SUBA <9 / BMI", { 0x96, 0x08, 0x90, 0x09, 0x2B, 0xFA }, 6, 3, 11 },
    { "LDA ,X / STA ,Y / BRA",  { 0xA6, 0x84, 0xA7, 0xA4, 0x20, 0xFA }, 6, 3, 11 },
    { "NOP / BRA",              { 0x12, 0x20, 0xFD },                   3, 2,  5 },
};

enum { MEM_DRAM, MEM_PSRAM, MEM_PSRAM_PRESSURE, MEM_COUNT };
static const char* const mem_names[MEM_COUNT] = { "dram", "psram", "psram+render" };

#define BENCH_CYCLES   300000      // emulated cycles per measurement
#define BENCH_CHUNK    114         // one fast-mode scanline
#define PRESSURE_BYTES 640         // PSRAM read per chunk, like a 640-px line

static uint32_t run_one(uint8_t variant, const BenchProg* p, uint8_t* page0, uint8_t* pageN,
                        const volatile uint8_t* pressure, uint32_t pressure_len, bool idle_skip = false) {
    memset(page0, 0, 0x2000);
    memset(pageN, 0, 0x2000);
    memcpy(page0 + 0x100, p->code, p->len);
    page0[0x09] = 1;                // LDA <8 / SUBA <9 stays negative
    bench_pages[0] = page0;
    for (int i = 1; i < 8; i++) bench_pages[i] = pageN;

    MC6809 cpu;
    mc6809_init(&cpu);
    cpu.read = bench_rd;
    cpu.write = bench_wr;
    cpu.variant = variant;
    cpu.rd_page = bench_pages;
    cpu.wr_page = bench_pages;
    cpu.fast_limit = 0xFF00;
    cpu.idle_skip = idle_skip;
    cpu.pc = 0x0100;
    cpu.x = 0xFFFF;                 // LEAX loop: 65535 passes before it falls through
    cpu.y = 0x4000;
    cpu.s = 0x1F00;

    // Only the run calls are timed; the PSRAM reads between them only
    // disturb the cache, as the renderer does between scanlines.
    uint32_t ran = 0, sink = 0, off = 0, cc = 0;
    while (ran < BENCH_CYCLES) {
        const uint32_t t0 = ccount();
        ran += mc6809_run_variant(&cpu, BENCH_CHUNK);
        cc += ccount() - t0;
        if (pressure) {
            for (uint32_t i = 0; i < PRESSURE_BYTES; i += 4) sink += pressure[off + i];
            off = (off + PRESSURE_BYTES) % (pressure_len - PRESSURE_BYTES);
        }
    }
    (void)sink;
    // host cycles per emulated cycle, x100
    return (uint32_t)((uint64_t)cc * 100 / ran);
}


// Dispatch kernel: a 32-way switch over a byte stream, like the
// interpreter's opcode switch. Built twice from the same source, once in
// flash and once in IRAM, to see what the flash cache costs a dispatch loop.
#define BENCH_KERNEL_BODY                                                    \
    uint32_t acc = 1, pc = 0;                                                \
    for (int i = 0; i < n; i++) {                                            \
        const uint8_t op = prog[pc];                                         \
        pc = (pc + 1) & 255;                                                 \
        switch (op) {                                                        \
        case 0:  acc += 1; break;          case 1:  acc ^= 0x55; break;      \
        case 2:  acc <<= 1; break;         case 3:  acc >>= 1; break;        \
        case 4:  acc += prog[acc & 255]; break;                              \
        case 5:  acc -= 3; break;          case 6:  acc |= 0x100; break;     \
        case 7:  acc &= 0xFFFF; break;     case 8:  acc += pc; break;        \
        case 9:  acc ^= pc; break;         case 10: acc += 7; break;         \
        case 11: acc -= pc; break;         case 12: acc = ~acc; break;       \
        case 13: acc += acc >> 3; break;   case 14: acc ^= acc << 2; break;  \
        case 15: acc += 11; break;         case 16: acc -= 13; break;        \
        case 17: acc ^= 0xAA; break;       case 18: acc += acc; break;       \
        case 19: acc |= 1; break;          case 20: acc &= ~1u; break;       \
        case 21: acc += 17; break;         case 22: acc -= 19; break;        \
        case 23: acc ^= 0x5A5A; break;     case 24: acc += 23; break;        \
        case 25: acc -= 29; break;         case 26: acc ^= i; break;         \
        case 27: acc += i; break;          case 28: acc -= i; break;         \
        case 29: acc ^= acc >> 5; break;   case 30: acc += 31; break;        \
        default: acc -= 37; break;                                           \
        }                                                                    \
    }                                                                        \
    return acc;

static uint32_t __attribute__((noinline)) kernel_flash(const uint8_t* prog, int n) { BENCH_KERNEL_BODY }
static uint32_t __attribute__((noinline)) IRAM_ATTR kernel_iram(const uint8_t* prog, int n) { BENCH_KERNEL_BODY }

static uint32_t __attribute__((noinline, optimize("no-jump-tables"))) kernel_nojt(const uint8_t* prog, int n) { BENCH_KERNEL_BODY }

// Same ops, dispatched through a label table in DRAM (computed goto).
#define K(x) L##x
static uint32_t __attribute__((noinline)) kernel_goto(const uint8_t* prog, int n) {
    static void* tab[32] = {
        &&L0, &&L1, &&L2, &&L3, &&L4, &&L5, &&L6, &&L7, &&L8, &&L9, &&L10, &&L11, &&L12, &&L13, &&L14, &&L15,
        &&L16, &&L17, &&L18, &&L19, &&L20, &&L21, &&L22, &&L23, &&L24, &&L25, &&L26, &&L27, &&L28, &&L29, &&L30, &&L31 };
    uint32_t acc = 1, pc = 0;
    int i = 0;
#define NEXT do { if (++i >= n) return acc; const uint8_t op = prog[pc]; pc = (pc + 1) & 255; goto *tab[op]; } while (0)
    { const uint8_t op = prog[pc]; pc = (pc + 1) & 255; goto *tab[op]; }
L0: acc += 1; NEXT;            L1: acc ^= 0x55; NEXT;
L2: acc <<= 1; NEXT;           L3: acc >>= 1; NEXT;
L4: acc += prog[acc & 255]; NEXT;
L5: acc -= 3; NEXT;            L6: acc |= 0x100; NEXT;
L7: acc &= 0xFFFF; NEXT;       L8: acc += pc; NEXT;
L9: acc ^= pc; NEXT;           L10: acc += 7; NEXT;
L11: acc -= pc; NEXT;          L12: acc = ~acc; NEXT;
L13: acc += acc >> 3; NEXT;    L14: acc ^= acc << 2; NEXT;
L15: acc += 11; NEXT;          L16: acc -= 13; NEXT;
L17: acc ^= 0xAA; NEXT;        L18: acc += acc; NEXT;
L19: acc |= 1; NEXT;           L20: acc &= ~1u; NEXT;
L21: acc += 17; NEXT;          L22: acc -= 19; NEXT;
L23: acc ^= 0x5A5A; NEXT;      L24: acc += 23; NEXT;
L25: acc -= 29; NEXT;          L26: acc ^= i; NEXT;
L27: acc += i; NEXT;           L28: acc -= i; NEXT;
L29: acc ^= acc >> 5; NEXT;    L30: acc += 31; NEXT;
L31: acc -= 37; NEXT;
#undef NEXT
}
#undef K

static void bench_kernel(void) {
    static uint8_t prog[256];
    uint32_t r = 12345;
    for (int i = 0; i < 256; i++) { r = r * 1103515245u + 12345u; prog[i] = (r >> 16) & 31; }
    const int n = 200000;
    uint32_t sink = kernel_flash(prog, 1000) + kernel_iram(prog, 1000);   // warm up
    uint32_t t0 = ccount();
    sink += kernel_flash(prog, n);
    const uint32_t cf = ccount() - t0;
    t0 = ccount();
    sink += kernel_iram(prog, n);
    const uint32_t ci = ccount() - t0;
    sink += kernel_nojt(prog, 1000) + kernel_goto(prog, 1000);
    t0 = ccount();
    sink += kernel_nojt(prog, n);
    const uint32_t cn = ccount() - t0;
    t0 = ccount();
    sink += kernel_goto(prog, n);
    const uint32_t cg = ccount() - t0;
    Serial.printf("  dispatch kernel, host cycles per dispatch: switch/flash %u  switch/iram %u  no-jump-tables %u  goto/dram-table %u  (%u)\n",
                  (unsigned)(cf / n), (unsigned)(ci / n), (unsigned)(cn / n), (unsigned)(cg / n),
                  (unsigned)(sink & 1));
}

void perf_bench_run(void) {
    uint8_t* d0 = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t* d1 = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t* p0 = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_SPIRAM);
    uint8_t* p1 = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_SPIRAM);
    const uint32_t plen = 256 * 1024;        // larger than the cache: every line misses
    uint8_t* pr = (uint8_t*)heap_caps_malloc(plen, MALLOC_CAP_SPIRAM);
    if (!d0 || !d1 || !p0 || !p1 || !pr) {
        Serial.println("perf_bench: allocation failed");
    } else {
        memset(pr, 0x55, plen);
        Serial.println("\n=== perf_bench: host cycles per emulated instruction ===");
        static const struct { uint8_t v; const char* n; } cpus[] = {
            { CPU_VARIANT_MC6809, "6809" }, { CPU_VARIANT_HD6309, "6309" },
        };
        for (const auto& c : cpus) {
            for (const BenchProg& p : progs) {
                Serial.printf("  %s %-24s", c.n, p.name);
                for (int m = 0; m < MEM_COUNT; m++) {
                    uint8_t* a = (m == MEM_DRAM) ? d0 : p0;
                    uint8_t* b = (m == MEM_DRAM) ? d1 : p1;
                    uint32_t x100 = run_one(c.v, &p, a, b, m == MEM_PSRAM_PRESSURE ? pr : nullptr, plen);
                    uint32_t per_insn = x100 * p.cycles / p.insns / 100;
                    Serial.printf("  %s %4u", mem_names[m], (unsigned)per_insn);
                }
                Serial.println();
            }
        }
        // A short loop that is not idle: the skip must cost (almost) nothing.
        for (const auto& c : cpus) {
            const BenchProg& p = progs[0];      // LEAX -1,X / BNE
            uint32_t off = run_one(c.v, &p, d0, d1, nullptr, plen, false) * p.cycles / p.insns / 100;
            uint32_t on  = run_one(c.v, &p, d0, d1, nullptr, plen, true) * p.cycles / p.insns / 100;
            Serial.printf("  %s %s  idle_skip off %u  on %u\n", c.n, p.name, (unsigned)off, (unsigned)on);
        }
        bench_kernel();
        Serial.println("=== perf_bench end ===");
    }
    heap_caps_free(d0); heap_caps_free(d1); heap_caps_free(p0); heap_caps_free(p1); heap_caps_free(pr);
}

#endif
