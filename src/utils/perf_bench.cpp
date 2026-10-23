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
    uint8_t code[12];
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

// The music FIRQ's instructions, one kind per loop (MC6809 timing applies to
// the HD6309 in emulation mode too, so host cycles per instruction compare).
static const BenchProg progs_firq[] = {
    { "PSHS/PULS A,B,DP,X / BRA", { 0x34, 0x1E, 0x35, 0x1E, 0x20, 0xFA },       6, 3, 26 },
    { "TFR A,DP / BRA",           { 0x1F, 0x8B, 0x20, 0xFC },                   4, 2,  9 },
    { "LDD # / ADDD # / STD <",   { 0xCC, 0x00, 0x00, 0xC3, 0x01, 0x23,
                                    0xDD, 0x07, 0x20, 0xF6 },                  10, 4, 15 },
    { "ADDA < / RORA / BRA",      { 0x9B, 0x4D, 0x46, 0x20, 0xFB },             5, 3,  9 },
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


// ---------------------------------------------------------------------------
// Rick Dangerous's 6 kHz GIME-timer FIRQ, interpreter cost per interrupt.
// The handlers are the game's own instructions (rick_game_annotated.asm
// $E098-$E13A, rick_music_player.asm $8000-$8070); the song is a made-up
// 3-row loop. The main loop is SYNC / BRA, so nearly all host time is the
// FIRQ. A FIRQ is raised every 298.6 CPU cycles (167 us at 1.79 MHz), at
// scanline-sized run boundaries as in the emulator; reading $FF93 acks it.
// I/O goes through a trivial callback here, so the machine's real register
// paths ($FF20, $FF91, $FF93) are not included.
// ---------------------------------------------------------------------------
static const uint8_t rick_sfx_firq[] = {     // $E098-$E13A
    0x97,0xC8,0xB6,0xFF,0x93,0x86,0x80,0x20,0x1E,0x97,0xC8,0xB6,0xFF,0x93,0x86,0x21,
    0xB7,0xFF,0x91,0x0C,0xBE,0x26,0x0D,0x0C,0xBD,0x2A,0x09,0x86,0x98,0xB7,0xFE,0xF5,
    0x86,0x80,0x20,0x03,0xB6,0xE0,0x00,0xB7,0xFF,0x20,0x86,0x20,0xB7,0xFF,0x91,0x86,
    0xFF,0x3B,0x97,0xF8,0xB6,0xFF,0x93,0x86,0x21,0xB7,0xFF,0x91,0x0C,0xEA,0x26,0x04,
    0x0C,0xE9,0x2B,0x1E,0x0C,0xED,0x26,0x08,0x0C,0xEC,0x96,0xEC,0x81,0xE0,0x27,0x40,
    0xB6,0x7F,0xFF,0xBB,0xDF,0xFF,0x46,0xB7,0xFF,0x20,0x86,0x20,0xB7,0xFF,0x91,0x86,
    0xFF,0x3B,0x96,0xF8,0x97,0xC8,0x0C,0xED,0x26,0x08,0x0C,0xEC,0x96,0xEC,0x81,0xE0,
    0x27,0xA9,0x96,0xEC,0x80,0x60,0x97,0xBD,0x96,0xED,0x97,0xBE,0x96,0x06,0xB7,0xFF,
    0xA9,0x4C,0xB7,0xFF,0xAA,0x4C,0xB7,0xFF,0xAB,0x86,0xA1,0xB7,0xFE,0xF5,0x20,0x94,
    0x96,0xF8,0x97,0xC8,0x96,0xE9,0x97,0xBD,0x96,0xEA,0x97,0xBE,0x86,0xA1,0xB7,0xFE,
    0xF5,0x20,0x81,
};
static const uint8_t rick_music_firq[] = {   // $8000-$8070
    0x34,0x1E,0x86,0x80,0x1F,0x8B,0xCC,0x00,0x00,0xC3,0x00,0x00,0xDD,0x07,0x97,0x4D,
    0xCC,0x00,0x00,0xC3,0x00,0x00,0xDD,0x11,0x9B,0x4D,0x46,0x97,0x27,0xCC,0x00,0x00,
    0xC3,0x00,0x00,0xDD,0x1E,0x44,0x8B,0x00,0x46,0xB7,0xFF,0x20,0x0A,0x4E,0x26,0x17,
    0x96,0x4F,0x88,0x01,0x97,0x4F,0x84,0x01,0x8B,0x5B,0x97,0x4E,0xDC,0x50,0x83,0x00,
    0x01,0xDD,0x50,0x26,0x02,0x8D,0x0B,0xB6,0xFF,0x93,0x35,0x1E,0x3B,0x00,0x5C,0x01,
    0x00,0x01,0x8E,0x81,0x25,0xEC,0x81,0x26,0x04,0x9E,0x70,0xEC,0x81,0xDD,0x50,0xEC,
    0x81,0xDD,0x0A,0xEC,0x81,0xDD,0x14,0xEC,0x81,0xDD,0x21,0x30,0x02,0x9F,0x53,0x39,
    0x81,
};
// [duration][step1][step2][step3][unused], duration 0 = back to <$70
static const uint8_t bench_song[] = {
    0x00,0x07, 0x08,0x5B, 0x06,0x42, 0x00,0x00, 0x00,0x00,
    0x00,0x14, 0x08,0x5B, 0x06,0x42, 0x04,0x2D, 0x00,0x00,
    0x00,0x0A, 0x10,0xB5, 0x0C,0x84, 0x04,0x2D, 0x00,0x00,
    0x00,0x00,
};

static MC6809* firq_cpu;
static uint8_t firq_rd(uint16_t a) {
    if (a == 0xFF93) firq_cpu->firq_pending = false;     // acknowledge
    return bench_pages[a >> 13][a & 0x1FFF];
}

enum { FIRQ_NONE, FIRQ_IDLE, FIRQ_ONE, FIRQ_TWO, FIRQ_MUSIC, FIRQ_KIND_COUNT };
static const char* const firq_names[FIRQ_KIND_COUNT] = {
    "no FIRQ (SYNC only)", "sfx idle ($98)", "sfx 1 channel ($A1)", "sfx 2 channels ($CA)", "music 3 voices",
};

// Host cycles per FIRQ (or per 1000 emulated cycles for FIRQ_NONE).
static uint32_t firq_run(uint8_t variant, uint8_t md, int kind, uint8_t* low, uint8_t* mid,
                         uint8_t* music, uint8_t* game, uint32_t* fires_out) {
    memset(low, 0, 0x2000); memset(mid, 0x40, 0x2000);
    memset(music, 0, 0x2000); memset(game, 0, 0x2000);
    low[0x100] = 0x13; low[0x101] = 0x20; low[0x102] = 0xFD;   // SYNC / BRA *-3
    memcpy(game + 0x0098, rick_sfx_firq, sizeof(rick_sfx_firq));
    memcpy(music, rick_music_firq, sizeof(rick_music_firq));
    memcpy(music + 0x125, bench_song, sizeof(bench_song));
    music[0x70] = 0x81; music[0x71] = 0x25;                    // <$70: song start
    game[0x1FF6] = 0xFE; game[0x1FF7] = 0xF4;                  // FIRQ vector -> $FEF4
    if (kind == FIRQ_MUSIC) {
        game[0x1EF4] = 0x7E; game[0x1EF5] = 0x80; game[0x1EF6] = 0x00;   // JMP $8000
    } else {
        static const uint8_t state[FIRQ_KIND_COUNT] = { 0x98, 0x98, 0xA1, 0xCA, 0 };
        game[0x1EF4] = 0x0E; game[0x1EF5] = state[kind];                 // JMP <$xx
    }
    game[0xBD] = 0x20; game[0xBE] = 0x00;      // 1-channel pointer $2000
    game[0xE9] = 0x20; game[0xEA] = 0x00;      // 2-channel: ch1 $2000
    game[0xEC] = 0x80; game[0xED] = 0x00;      //            ch2 $8000
    bench_pages[0] = low;
    for (int i = 1; i < 8; i++) bench_pages[i] = mid;
    bench_pages[4] = music;
    bench_pages[7] = game;

    MC6809 cpu;
    mc6809_init(&cpu);
    firq_cpu = &cpu;
    cpu.read = firq_rd;
    cpu.write = bench_wr;
    cpu.variant = variant;
    cpu.rd_page = bench_pages;
    cpu.wr_page = bench_pages;
    cpu.fast_limit = 0xFF00;
    cpu.idle_skip = true;
    cpu.pc = 0x0100; cpu.s = 0x1F00; cpu.dp = 0xE0;
    cpu.cc = MC6809_FLAG_I;      // IRQ masked, FIRQ open
    cpu.md = md;

    uint32_t ran = 0, cc = 0, fires = 0, acc_x10 = 0;
    while (ran < BENCH_CYCLES) {
        if (kind != FIRQ_NONE) {
            acc_x10 += BENCH_CHUNK * 10;
            if (acc_x10 >= 2986) { acc_x10 -= 2986; cpu.firq_pending = true; fires++; }
        }
        const uint32_t t0 = ccount();
        ran += mc6809_run_variant(&cpu, BENCH_CHUNK);
        cc += ccount() - t0;
    }
    *fires_out = fires;
    return kind == FIRQ_NONE ? (uint32_t)((uint64_t)cc * 1000 / ran) : cc / (fires ? fires : 1);
}

static void bench_firq(uint8_t* low, uint8_t* mid, uint8_t* music, uint8_t* game) {
    Serial.println("  Rick 6 kHz FIRQ, host cycles per interrupt (interpreter only; 100 per frame at 60 FPS):");
    static const struct { uint8_t v, md; const char* n; } cpus[] = {
        { CPU_VARIANT_MC6809, 0, "6809" }, { CPU_VARIANT_HD6309, 0, "6309 emul" },
        { CPU_VARIANT_HD6309, 1, "6309 native" },
    };
    for (const auto& c : cpus) {
        for (int k = 0; k < FIRQ_KIND_COUNT; k++) {
            uint32_t fires;
            uint32_t v = firq_run(c.v, c.md, k, low, mid, music, game, &fires);
            if (k == FIRQ_NONE)
                Serial.printf("    %-12s %-22s %5u host cycles per 1000 emulated\n", c.n, firq_names[k], (unsigned)v);
            else
                Serial.printf("    %-12s %-22s %5u  (= %4.2f ms per frame)  [%u FIRQs]\n", c.n, firq_names[k],
                              (unsigned)v, v * 100 / 240000.0f, (unsigned)fires);
        }
    }
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
        Serial.println("  music-FIRQ instructions, host cycles per instruction (6309 in emulation mode):");
        for (const BenchProg& p : progs_firq) {
            uint32_t a = run_one(CPU_VARIANT_MC6809, &p, d0, d1, nullptr, plen) * p.cycles / p.insns / 100;
            uint32_t b = run_one(CPU_VARIANT_HD6309, &p, d0, d1, nullptr, plen) * p.cycles / p.insns / 100;
            Serial.printf("    %-26s 6809 %4u  6309 %4u\n", p.name, (unsigned)a, (unsigned)b);
        }
        // A short loop that is not idle: the skip must cost (almost) nothing.
        for (const auto& c : cpus) {
            const BenchProg& p = progs[0];      // LEAX -1,X / BNE
            uint32_t off = run_one(c.v, &p, d0, d1, nullptr, plen, false) * p.cycles / p.insns / 100;
            uint32_t on  = run_one(c.v, &p, d0, d1, nullptr, plen, true) * p.cycles / p.insns / 100;
            Serial.printf("  %s %s  idle_skip off %u  on %u\n", c.n, p.name, (unsigned)off, (unsigned)on);
        }
        {
            uint8_t* music = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_SPIRAM);
            uint8_t* game  = (uint8_t*)heap_caps_malloc(0x2000, MALLOC_CAP_SPIRAM);
            if (music && game) bench_firq(p0, p1, music, game);
            heap_caps_free(music); heap_caps_free(game);
        }
        bench_kernel();
        Serial.println("=== perf_bench end ===");
    }
    heap_caps_free(d0); heap_caps_free(d1); heap_caps_free(p0); heap_caps_free(p1); heap_caps_free(pr);
}

#endif
