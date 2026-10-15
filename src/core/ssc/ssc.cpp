/*
 * ============================================================
 *        CoCo 2&3 Emulator for ESP32-TTGO-VGA32-COCO
 *   (C) 2026 Reinaldo Torres / CoCo Byte Club
 *   https://github.com/reyco2000/TTGO-VGA32-COCO
 *   Based on XRoar , co-developed with Claude Code
 *   GPL-3.0-or-later License
 * ============================================================
 *  File   : ssc.cpp
 *  Module : Speech/Sound Cartridge — TMS7040 + SP0256-AL2 + AY-3-8913
 * ============================================================
*/

// Provenance: VCC ssc/ssc_core.cpp (commit db8f7d7, 780TECH); port and bit
// wiring from MAME's src/devices/bus/coco/coco_ssc.cpp (license:
// BSD-3-Clause, copyright tim lindner).
//
// Changes for the ESP32:
//  - one instance, static state; the chips call in through ssc_bus.h;
//  - fixed tick rate (one call per scanline, 15720 Hz) with the clock ratios
//    kept as exact integer fractions, so there is no double;
//  - the TMS7040 runs its full cycle budget: cycles an instruction overshoots
//    one scanline are taken from the next (VCC drops them);
//  - integer mix and sound-activity filter;
//  - nothing is allocated until ssc_load_roms(): with the cartridge off it
//    costs no RAM. The TMS7040 (hot) goes to internal RAM, the SP0256 and
//    the 2 KB static RAM to PSRAM.
//
// This file stores to PSRAM, so it keeps the PSRAM memw workaround.

#pragma GCC optimize("O2", "jump-tables")
#include "ssc.h"
#include "ssc_bus.h"
#include "tms7000.h"
#include "ay8913.h"
#include "sp0256.h"
#include "../sound.h"
#include <new>
#include <stdlib.h>
#include <string.h>

#ifdef ARDUINO
#include <esp_heap_caps.h>
#include "../../../config.h"
#include "../../hal/hal.h"
#include "../../utils/debug.h"
#define SSC_ALLOC_FAST(n)   heap_caps_malloc((n), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define SSC_ALLOC_SLOW(n)   heap_caps_malloc((n), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
#define SSC_ALLOC_FAST(n)   malloc(n)
#define SSC_ALLOC_SLOW(n)   malloc(n)
#define DEBUG_PRINTF(...)   ((void)0)
#endif

using namespace ssc;

bool g_ssc_enabled = false;

// Port C bit assignments, from MAME's coco_ssc.cpp. RAM A8-A10 share their
// positions with BC1/BDIR/CS: the RAM address is port C:port B masked to 11 bits.
static const uint8_t C_BC1 = 0x01;
static const uint8_t C_RRW = 0x08;
static const uint8_t C_BDR = 0x08;
static const uint8_t C_RCS = 0x10;
static const uint8_t C_ALD = 0x20;
static const uint8_t C_ACS = 0x40;
static const uint8_t C_BSY = 0x80;

// Clocks per scanline, as exact fractions of the 15720 Hz tick.
static const uint32_t TICK_HZ   = 15720;      // 262 lines x 60 frames
static const uint32_t TMS_HZ    = 894886;     // 1.789772 MHz input, /2 internal
static const uint32_t AY_HZ     = 1789772;    // internal step = clock / 8
static const uint32_t AY_DIV    = 8 * TICK_HZ;
static const uint32_t SPO_HZ    = 10000;      // 3.12 MHz / 312

// Sound Activity Circuit (cocossc_sac_device), in Q24. Per-tick
// coefficients as VCC uses them at the scanline rate.
static const int32_t SAC_HPF_ALPHA = 16609444;  // 0.99
static const int32_t SAC_ATTACK    = 43620;     // 0.0026
static const int32_t SAC_DECAY     = 5033;      // 0.0003
static const int32_t SAC_THRESH_ON = 838861;    // 0.05
static const int32_t SAC_THRESH_OFF= 167772;    // 0.01

// Scanlines run per batch. Between two scanlines the emulator's own loop
// evicts the chips' code from the flash cache; running them for several
// scanlines back to back cut their cost by ~30% on the device. A batch runs the chips for SSC_BATCH scanlines back to
// back into s_batch[], and each scanline then only plays one sample. The
// cartridge sees the CoCo's writes, and the CoCo sees BUSY drop, up to
// SSC_BATCH scanlines later (8 = 0.5 ms). Host tests build with 1 to compare
// tick for tick with VCC.
#ifndef SSC_BATCH
#define SSC_BATCH 8
#endif

// Output gains (VCC: AY x2.0, SP0256 x1.75, full scale 127), applied as
// sample * gain / 32768.
static const int32_t MIX_AY_GAIN  = 254;
static const int32_t MIX_SPO_GAIN = 222;

static Tms7000* s_cpu = nullptr;     // internal RAM
static Sp0256*  s_spo = nullptr;     // PSRAM
static uint8_t* s_ram = nullptr;     // 2 KB static RAM, PSRAM
static Ay8913   s_ay;

// $FF7D/$FF7E glue state
static uint8_t s_reset_line  = 1;
static bool    s_tms_busy    = false;
static uint8_t s_porta_latch = 0;
static uint8_t s_portb_latch = 0;
static uint8_t s_portc_latch = 0xff;
static uint8_t s_portd_latch = 0;

// SAC state, Q24
static int32_t s_sac_prev_in  = 0;
static int32_t s_sac_prev_out = 0;
static int32_t s_sac_env      = 0;
static bool    s_sac_active   = false;

// Clock-domain accumulators
static uint32_t s_cpu_acc  = 0;      // TMS_HZ per tick, units of 1/TICK_HZ cycle
static int32_t  s_cpu_debt = 0;      // cycles already run into this scanline
static uint32_t s_ay_acc   = 0;
static uint32_t s_spo_acc  = 0;
static int16_t  s_spo_sample = 0;
static int8_t   s_level      = 0;      // last sample played
static int8_t   s_batch[SSC_BATCH];
static int      s_batch_pos  = 0;      // next sample of s_batch to play; 0 = run a batch

static uint32_t s_ay_write_count   = 0;
static uint32_t s_ald_speech_count = 0;
static uint32_t s_command_count    = 0;

// ---------------------------------------------------------------------
// ROMs and allocation
// ---------------------------------------------------------------------

bool ssc_load_roms_from(const uint8_t* pic, size_t pic_len,
                        const uint8_t* speech, size_t speech_len) {
    if (s_cpu) return true;
    if (!pic || pic_len != 4096 || !speech || speech_len != 2048) return false;

    void* cpu_mem = SSC_ALLOC_FAST(sizeof(Tms7000));
    void* spo_mem = SSC_ALLOC_SLOW(sizeof(Sp0256));
    uint8_t* ram  = (uint8_t*)SSC_ALLOC_SLOW(2048);
    if (!cpu_mem || !spo_mem || !ram) {
        free(cpu_mem); free(spo_mem); free(ram);
        DEBUG_PRINTF("SSC: out of memory");
        return false;
    }
    Tms7000* cpu = new (cpu_mem) Tms7000();
    Sp0256*  spo = new (spo_mem) Sp0256();
    cpu->LoadRom(pic, pic_len);
    spo->LoadRom(speech, speech_len);
    s_spo = spo;
    s_ram = ram;
    s_cpu = cpu;
    ssc_reset();
    return true;
}

#ifdef ARDUINO
// Loads one ROM file of exactly `size` bytes into a PSRAM buffer.
static uint8_t* load_rom_file(const char* name, size_t size) {
    char path[64];
    snprintf(path, sizeof(path), "%s/%s", ROM_BASE_PATH, name);
    if (hal_storage_file_size(path) != size) {
        DEBUG_PRINTF("SSC: %s missing or not %u bytes", path, (unsigned)size);
        return nullptr;
    }
    uint8_t* buf = (uint8_t*)SSC_ALLOC_SLOW(size);
    if (buf && !hal_storage_load_file(path, buf, size)) {
        free(buf);
        buf = nullptr;
    }
    return buf;
}

bool ssc_load_roms(void) {
    if (s_cpu) return true;
    uint8_t* pic    = load_rom_file(ROM_SSC_PIC_FILE, 4096);
    uint8_t* speech = load_rom_file(ROM_SSC_SPEECH_FILE, 2048);
    bool ok = ssc_load_roms_from(pic, pic ? 4096 : 0, speech, speech ? 2048 : 0);
    free(pic);
    free(speech);
    if (ok) DEBUG_PRINTF("SSC: Speech/Sound Cartridge ready at $FF7D/$FF7E");
    return ok;
}
#endif

// ---------------------------------------------------------------------
// Reset and host ports
// ---------------------------------------------------------------------

void ssc_reset(void) {
    if (!s_cpu) return;
    s_reset_line  = 1;
    s_tms_busy    = false;
    s_porta_latch = 0;
    s_portb_latch = 0;
    s_portc_latch = 0xff;
    s_portd_latch = 0;
    s_cpu_acc = 0;
    s_cpu_debt = 0;
    s_ay_acc = 0;
    s_spo_acc = 0;
    s_spo_sample = 0;
    s_level = 0;
    s_batch_pos = 0;
    s_sac_prev_in = s_sac_prev_out = s_sac_env = 0;
    s_sac_active = false;
    memset(s_ram, 0, 2048);

    s_cpu->Reset();
    s_ay.Reset();
    s_spo->Reset();
    sound_set_ssc(0);
}

uint8_t ssc_read(uint16_t addr) {
    if (addr != SSC_DATA_ADDR || !s_cpu) return 0xFF;
    uint8_t data = 0x1f;
    if (!s_tms_busy)     data |= SSC_ST_READY;
    if (s_spo->Standby()) data |= SSC_ST_STANDBY;
    if (!s_sac_active)   data |= SSC_ST_QUIET;
    return data;
}

void ssc_write(uint16_t addr, uint8_t val) {
    if (!s_cpu) return;
    if (addr == SSC_CTRL_ADDR) {
        if (val & 1) s_spo->Reset();
        if ((s_reset_line & 1) && !(val & 1)) {
            s_cpu->Reset();
            s_ay.Reset();
            s_tms_busy = false;
        }
        s_reset_line = val;
    } else if (addr == SSC_DATA_ADDR) {
        s_porta_latch = val;
        s_tms_busy = true;
        s_command_count++;
        s_cpu->SetIrqLine(Tms7000Irq::Int3, true);
    }
}

// ---------------------------------------------------------------------
// TMS7040 port wiring (ssc_bus.h)
// ---------------------------------------------------------------------

uint8_t ssc_bus_porta_read(void) {
    s_cpu->SetIrqLine(Tms7000Irq::Int3, false);
    return s_porta_latch;
}

void ssc_bus_portb_write(uint8_t data) {
    s_portb_latch = data;
}

uint8_t ssc_bus_portc_read(void) {
    return s_portc_latch;
}

void ssc_bus_portc_write(uint8_t data) {
    if ((data & C_RCS) == 0 && (data & C_RRW) == 0) {           // static RAM write
        uint16_t address = (uint16_t)((data << 8) | s_portb_latch) & 0x7ff;
        s_ram[address] = s_portd_latch;
    }

    if ((data & C_ACS) == 0) {                                   // AY selected
        if ((data & (C_BDR | C_BC1)) == (C_BDR | C_BC1))         // latch address
            s_ay.SelectRegister(s_portd_latch);
        if ((data & C_BDR) && !(data & C_BC1)) {                 // write data
            s_ay.WriteData(s_portd_latch);
            s_ay_write_count++;
        }
    }

    // ALD strobe on the falling edge. Codes 0-4 are pauses, which the firmware
    // strobes continuously while idle; 5-63 are real allophones.
    if ((s_portc_latch & C_ALD) && !(data & C_ALD) && s_portd_latch < 64) {
        s_spo->Ald(s_portd_latch);
        if (s_portd_latch >= 5) s_ald_speech_count++;
    }

    if (!(s_portc_latch & C_BSY) && (data & C_BSY))
        s_tms_busy = false;                                      // BUSY released

    s_portc_latch = data;
}

uint8_t ssc_bus_portd_read(void) {
    if (!(s_portc_latch & C_RCS) && (s_portc_latch & C_RRW)) {
        uint16_t address = (uint16_t)((s_portc_latch << 8) | s_portb_latch) & 0x7ff;
        s_portd_latch = s_ram[address];
    }
    if (!(s_portc_latch & C_ACS)) {
        if (!(s_portc_latch & C_BDR) && (s_portc_latch & C_BC1))
            s_portd_latch = s_ay.ReadData();
    }
    return s_portd_latch;
}

void ssc_bus_portd_write(uint8_t data) {
    s_portd_latch = data;
}

void ssc_bus_speech_drq(bool ready) {
    // Guard for a host test driving an Sp0256 with no TMS7040 allocated.
    if (s_cpu) s_cpu->SetIrqLine(Tms7000Irq::Int1, ready);
}

// ---------------------------------------------------------------------
// Per-scanline tick
// ---------------------------------------------------------------------

static int64_t q24_mul(int32_t a, int32_t b) {
    return ((int64_t)a * b) >> 24;
}

// High-pass filter, then an asymmetric attack/decay envelope follower with
// hysteresis. x is the AY sample in Q24 (sample / 32768).
static void sound_activity(int32_t x) {
    int32_t y = (int32_t)q24_mul(SAC_HPF_ALPHA, s_sac_prev_out + x - s_sac_prev_in);
    s_sac_prev_in  = x;
    s_sac_prev_out = y;

    int32_t rect = y < 0 ? -y : y;
    int32_t coef = (rect > s_sac_env) ? SAC_ATTACK : SAC_DECAY;
    s_sac_env += (int32_t)q24_mul(rect - s_sac_env, coef);

    if (s_sac_active && s_sac_env < SAC_THRESH_OFF) s_sac_active = false;
    else if (s_sac_env > SAC_THRESH_ON)             s_sac_active = true;
}

// One scanline of the three chips; returns the mixed sample.
static int8_t run_scanline(void) {
    // TMS7040: whole cycles owed this scanline, minus what the last
    // instruction of the previous scanline already ran past its end.
    s_cpu_acc += TMS_HZ;
    int32_t cycles = (int32_t)(s_cpu_acc / TICK_HZ);
    s_cpu_acc -= (uint32_t)cycles * TICK_HZ;
    cycles -= s_cpu_debt;
    if (cycles > 0) s_cpu_debt = s_cpu->Execute(cycles) - cycles;
    else            s_cpu_debt = -cycles;

    // AY-3-8913
    s_ay_acc += AY_HZ;
    int steps = (int)(s_ay_acc / AY_DIV);
    s_ay_acc -= (uint32_t)steps * AY_DIV;
    int16_t ay = s_ay.Render(steps);

    // SP0256 at its own ~10 kHz
    s_spo_acc += SPO_HZ;
    while (s_spo_acc >= TICK_HZ) {
        s_spo->GenerateSamples(&s_spo_sample, 1);
        s_spo_acc -= TICK_HZ;
    }

    sound_activity((int32_t)ay << 9);

    int32_t mix = (int32_t)ay * MIX_AY_GAIN + (int32_t)s_spo_sample * MIX_SPO_GAIN;
    mix = (mix + 16384) >> 15;
    if (mix >  127) mix =  127;
    if (mix < -127) mix = -127;
    return (int8_t)mix;
}

void ssc_scanline_tick(void) {
    if (!s_cpu) return;
    if (s_batch_pos == 0) {
        for (int i = 0; i < SSC_BATCH; i++) s_batch[i] = run_scanline();
    }
    int8_t v = s_batch[s_batch_pos];
    if (++s_batch_pos == SSC_BATCH) s_batch_pos = 0;
    if (v != s_level) {
        s_level = v;
        sound_set_ssc(v);
    }
}

// ---------------------------------------------------------------------
// Introspection
// ---------------------------------------------------------------------

bool     ssc_busy(void)             { return s_tms_busy; }
uint32_t ssc_command_count(void)    { return s_command_count; }
uint32_t ssc_ald_speech_count(void) { return s_ald_speech_count; }
uint32_t ssc_ay_write_count(void)   { return s_ay_write_count; }
int8_t   ssc_level(void)            { return s_level; }
