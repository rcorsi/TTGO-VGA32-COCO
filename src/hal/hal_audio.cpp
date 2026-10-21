/*
 * ============================================================
 *        CoCo 2&3 Emulator for ESP32-TTGO-VGA32-COCO
 *   (C) 2026 Reinaldo Torres / CoCo Byte Club
 *   https://github.com/reyco2000/TTGO-VGA32-COCO
 *   Based on XRoar , co-developed with Claude Code
 *   GPL-3.0-or-later License
 * ============================================================
 *  File   : hal_audio.cpp
 *  Module : Audio HAL — ESP32 internal DAC1 (GPIO25) via timer ISR
 * ============================================================
*/

// Stores in this file go to internal RAM only (see the header for the rule).
#include "../utils/no_psram_memw.h"
#include "hal.h"
#include "../utils/debug.h"

#define SCANLINES_PER_FRAME  262
#define SAMPLES_PER_FRAME    SCANLINES_PER_FRAME

// The timer counts whole microseconds, so the ISR runs every 45 us
// (22 222 Hz), not at exactly 22 050 Hz.
#define AUDIO_ISR_PERIOD_US  45
// Same integer division as pace_frame() in the sketch.
#define FRAME_PERIOD_US      (1000000 / TARGET_FPS)

// Samples the ISR consumes per tick, Q16: one frame of samples per frame period.
#define STRIDE_NOMINAL_Q16   ((uint32_t)(((uint64_t)SAMPLES_PER_FRAME * 65536 * AUDIO_ISR_PERIOD_US) / FRAME_PERIOD_US))
// Rate control. The emulator is paced, so it never runs fast; it can run
// slow (a screen too heavy to hold 60 FPS), and playback then slows with it.
// Feed-forward: the ISR consumes one frame of samples per measured frame
// interval (averaged), so the rate follows a speed change within a few
// frames. A PI trim of at most +-5 % then holds the fill level at target.
#define STRIDE_FF_MIN        ((uint32_t)(STRIDE_NOMINAL_Q16 * 55 / 100)) // 55 % speed
#define STRIDE_FF_MAX        ((uint32_t)(STRIDE_NOMINAL_Q16 * 102 / 100))
#define STRIDE_TRIM          ((int32_t)(STRIDE_NOMINAL_Q16 / 20))        // +-5 %
// Frame intervals outside this range are pauses (OSD, debugger, SD) or
// noise, not the emulation speed, and are left out of the average.
#define FRAME_DT_MIN_US      8000
#define FRAME_DT_MAX_US      60000
#define FRAME_DT_AVG_SHIFT   3      // 8-frame average
// PI gains in stride units: 15/32 per Q4 unit of averaged error (7.5 per
// sample) and 5/64 per sample-frame of accumulated error.
#define RATE_KP_NUM          15
#define RATE_KP_DEN          32
#define RATE_KI_NUM          5
#define RATE_KI_DEN          64
// Most ISR periods made up in one run after the ISR was held off.
#define AUDIO_CATCHUP_MAX    64

// Ring buffer between the emulation loop (producer, one sample per scanline)
// and the timer ISR (consumer). The emulator delivers a frame's samples in a
// burst and then idles until the next frame slot, so the fill level swings by
// most of a frame; the target leaves room for that plus late frames.
#define AUDIO_RING_SIZE      2048                       // power of two
#define AUDIO_RING_MASK      (AUDIO_RING_SIZE - 1)
#define AUDIO_POS_MASK       ((AUDIO_RING_SIZE << 16) - 1)
// Fill wanted right after a frame: one frame plus 384 samples (17 ms), so a
// frame that takes two frame periods (33 ms) still does not drain the ring.
#define AUDIO_FILL_TARGET    (SAMPLES_PER_FRAME + 384)
#define AUDIO_FILL_LIMIT     (AUDIO_RING_SIZE - 64)     // producer drops samples above this

static uint8_t audio_ring[AUDIO_RING_SIZE];
static volatile uint32_t audio_wr = 0;                  // producer index (core 1 loop)
static volatile uint32_t audio_rd_q16 = 0;              // consumer position, Q16 (ISR)
static volatile uint32_t audio_stride_q16 = STRIDE_NOMINAL_Q16;
// True while the ring is empty: the ISR leaves the DAC at its last level and
// waits for hal_audio_commit_frame() to restart it with a full frame queued.
static volatile bool     audio_holding = true;
static int32_t           audio_err_q4 = 0;              // smoothed fill error, Q4
static int32_t           audio_err_sum = 0;             // accumulated fill error (integral term)
static uint32_t          audio_period_cyc = 0;          // ISR period in CPU clock cycles
static int32_t           audio_dt_avg_us = FRAME_PERIOD_US;   // averaged frame interval
static volatile uint8_t  audio_current_level = 0;

static hw_timer_t* audio_timer = nullptr;

// Diagnostics (see HalAudioStats in hal.h). Written by the ISR and by
// hal_audio_commit_frame(); read by the debug server on core 0.
static volatile HalAudioStats audio_stats;
static volatile uint32_t stat_last_isr  = 0;      // CCOUNT at previous ISR entry
static volatile bool     stat_have_isr  = false;
static int64_t           stat_last_commit_us = 0;

// ============================================================================
//  ESP32 internal DAC1 (GPIO25)
// ============================================================================

// ESP-IDF 5 still ships the legacy DAC driver (`driver/dac.h`) under a
// deprecation warning. We use it here because it matches the existing
// timer-ISR / sample-by-sample model 1:1 and avoids spinning up the full
// dac_continuous DMA pipeline for a single mono channel.
#include "driver/dac.h"
#include "hal/cpu_ll.h"      // cpu_ll_get_cycle_count(): inline, ISR-safe

#define AUDIO_DAC_CHANNEL    DAC_CHANNEL_1   // GPIO25

static void IRAM_ATTR audio_timer_isr() {
    uint32_t t0 = cpu_ll_get_cycle_count();
    // Periods elapsed since the last run. Normally 1; more when another
    // interrupt or a critical section held this one off, so playback keeps
    // real time instead of falling behind by the ticks that were lost.
    uint32_t periods = 1;
    if (stat_have_isr) {
        uint32_t gap = t0 - stat_last_isr;
        if (gap < audio_stats.isr_gap_cyc_min) audio_stats.isr_gap_cyc_min = gap;
        if (gap > audio_stats.isr_gap_cyc_max) audio_stats.isr_gap_cyc_max = gap;
        const uint32_t period = audio_period_cyc;
        while (gap > period + (period >> 1) && periods < AUDIO_CATCHUP_MAX) {
            gap -= period;
            periods++;
        }
    }
    stat_last_isr = t0;
    stat_have_isr = true;

    if (audio_holding) {
        audio_stats.hold_ticks++;
    } else {
        uint32_t pos  = audio_rd_q16;
        uint32_t idx  = pos >> 16;
        uint32_t fill = (audio_wr - idx) & AUDIO_RING_MASK;
        if (fill == 0) {
            // Ring ran dry (OSD open, debugger pause, a long SD access).
            audio_holding = true;
            audio_stats.underruns++;
        } else {
            dac_output_voltage(AUDIO_DAC_CHANNEL, audio_ring[idx]);
            uint32_t next = pos + audio_stride_q16 * periods;
            if ((next >> 16) - idx > fill)               // never pass the producer
                next = (idx + fill) << 16;
            audio_rd_q16 = next & AUDIO_POS_MASK;
            audio_stats.catchup_ticks += periods - 1;
        }
    }

    uint32_t dur = cpu_ll_get_cycle_count() - t0;
    if (dur > audio_stats.isr_cyc_max) audio_stats.isr_cyc_max = dur;
}

void hal_audio_init(void) {
    dac_output_enable(AUDIO_DAC_CHANNEL);
    // Idle level is 0: the sound core's output is unipolar like the real
    // hardware (silence = 0 V); the jack's AC coupling removes any DC.
    dac_output_voltage(AUDIO_DAC_CHANNEL, 0);
    memset(audio_ring, 0, AUDIO_RING_SIZE);
    hal_audio_reset_stats();
    audio_period_cyc = (uint32_t)getCpuFrequencyMhz() * AUDIO_ISR_PERIOD_US;
    // ESP32-Arduino 2.x timer API: timerBegin(timer_num, divider, countUp).
    // APB is 80 MHz; divider 80 → 1 MHz timer tick (1 µs per tick).
    audio_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(audio_timer, audio_timer_isr, true);
    timerAlarmWrite(audio_timer, AUDIO_ISR_PERIOD_US, true);
    timerAlarmEnable(audio_timer);
    audio_current_level = 0;
    DEBUG_PRINTF("  Audio: DAC1 (GPIO25), %d us ISR, stride Q16=%u",
                 AUDIO_ISR_PERIOD_US, (unsigned)STRIDE_NOMINAL_Q16);
}

// ============================================================================
//  Backend-independent surface
// ============================================================================

void hal_audio_write_sample(int16_t left, int16_t right) {
    (void)left; (void)right;
}

void hal_audio_set_volume(uint8_t volume) {
    DEBUG_PRINTF("  Audio: volume set to %d", volume);
}

static inline uint32_t audio_fill(void) {
    return (audio_wr - (audio_rd_q16 >> 16)) & AUDIO_RING_MASK;
}

void hal_audio_capture_scanline(void) {
    if (audio_fill() >= AUDIO_FILL_LIMIT) {
        audio_stats.overruns++;
        return;
    }
    uint32_t wr = audio_wr;
    audio_ring[wr & AUDIO_RING_MASK] = audio_current_level;
    audio_wr = wr + 1;
}

// Frame end: the ring is at its fullest here, so this is where the fill level
// is judged. The ISR rate is set from the measured frame interval (feed-
// forward: one frame of samples per interval) plus a small PI trim that
// holds AUDIO_FILL_TARGET. So when the emulator runs below 60 FPS the sound
// slows with it at once, instead of the ring draining (gaps) or, when it
// speeds up again, overflowing (dropped samples) while a slow loop catches up.
void hal_audio_commit_frame(void) {
    uint32_t fill = audio_fill();

    int64_t now = esp_timer_get_time();
    if (stat_last_commit_us != 0) {
        uint32_t dt = (uint32_t)(now - stat_last_commit_us);
        if (dt < audio_stats.commit_us_min) audio_stats.commit_us_min = dt;
        if (dt > audio_stats.commit_us_max) audio_stats.commit_us_max = dt;
        if (dt >= FRAME_DT_MIN_US && dt <= FRAME_DT_MAX_US)
            audio_dt_avg_us += ((int32_t)dt - audio_dt_avg_us) >> FRAME_DT_AVG_SHIFT;
    }
    stat_last_commit_us = now;
    audio_stats.commits++;

    uint32_t ff = (uint32_t)(((uint64_t)SAMPLES_PER_FRAME * 65536 * AUDIO_ISR_PERIOD_US)
                             / (uint32_t)audio_dt_avg_us);
    if (ff < STRIDE_FF_MIN) ff = STRIDE_FF_MIN;
    if (ff > STRIDE_FF_MAX) ff = STRIDE_FF_MAX;

    int32_t trim = 0;
    if (audio_holding) {
        // Restart playback once a whole frame is queued. The ISR does not
        // touch the read position while holding.
        if (fill >= SAMPLES_PER_FRAME) {
            audio_err_q4 = 0;
            audio_err_sum = 0;
            audio_holding = false;
        }
    } else {
        int32_t err = (int32_t)fill - AUDIO_FILL_TARGET;
        audio_err_q4 += ((err << 4) - audio_err_q4) >> 4;      // ~16-frame average
        // The integral term removes a lasting offset the feed-forward leaves
        // (timer vs frame clock), so the fill level returns to the target.
        const int32_t sum_lim = STRIDE_TRIM * RATE_KI_DEN / RATE_KI_NUM;
        audio_err_sum += err;
        if (audio_err_sum > sum_lim) audio_err_sum = sum_lim;
        if (audio_err_sum < -sum_lim) audio_err_sum = -sum_lim;
        trim = (audio_err_q4 * RATE_KP_NUM) / RATE_KP_DEN
             + (audio_err_sum * RATE_KI_NUM) / RATE_KI_DEN;
        if (trim > STRIDE_TRIM) trim = STRIDE_TRIM;
        if (trim < -STRIDE_TRIM) trim = -STRIDE_TRIM;
    }
    audio_stride_q16 = (uint32_t)((int32_t)ff + trim);
    audio_stats.stride_adj = (int32_t)audio_stride_q16 - (int32_t)STRIDE_NOMINAL_Q16;

    audio_stats.fill = fill;
    if (fill < audio_stats.fill_min) audio_stats.fill_min = fill;
    if (fill > audio_stats.fill_max) audio_stats.fill_max = fill;
}

void hal_audio_get_stats(HalAudioStats* out) {
    out->commits         = audio_stats.commits;
    out->underruns       = audio_stats.underruns;
    out->hold_ticks      = audio_stats.hold_ticks;
    out->catchup_ticks   = audio_stats.catchup_ticks;
    out->overruns        = audio_stats.overruns;
    out->fill            = audio_stats.fill;
    out->fill_min        = audio_stats.fill_min;
    out->fill_max        = audio_stats.fill_max;
    out->stride_adj      = audio_stats.stride_adj;
    out->commit_us_min   = audio_stats.commit_us_min;
    out->commit_us_max   = audio_stats.commit_us_max;
    out->isr_cyc_max     = audio_stats.isr_cyc_max;
    out->isr_gap_cyc_min = audio_stats.isr_gap_cyc_min;
    out->isr_gap_cyc_max = audio_stats.isr_gap_cyc_max;
}

// Not locked against the ISR: a counter may be one event off right after.
void hal_audio_reset_stats(void) {
    audio_stats.commits = audio_stats.underruns = 0;
    audio_stats.hold_ticks = audio_stats.overruns = 0;
    audio_stats.catchup_ticks = 0;
    audio_stats.fill_min        = UINT32_MAX;
    audio_stats.fill_max        = 0;
    audio_stats.commit_us_min   = UINT32_MAX;
    audio_stats.commit_us_max   = 0;
    audio_stats.isr_cyc_max     = 0;
    audio_stats.isr_gap_cyc_min = UINT32_MAX;
    audio_stats.isr_gap_cyc_max = 0;
}

void hal_audio_set_level(uint8_t level) {
    audio_current_level = level;
}

void hal_audio_debug_tick(void) {}
