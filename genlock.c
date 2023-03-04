#include <stdio.h>
#include <stdlib.h>
#include "pico/scanvideo.h"
#include "hardware/clocks.h"
#include "genlock.pio.h"
#include "genlock.h"

// Custom screen mode with 524 lines to better match the Atom's 6847 (262 lines)
static scanvideo_timing_t custom_timing = {
    .clock_freq = SYS_FREQ * 100,
    .h_active = 640,
    .v_active = 480,
    .h_front_porch = 16,
    .h_pulse = 64,
    .h_total = 800,
    .h_sync_polarity = 1,
    .v_front_porch = 1,
    .v_pulse = 2,
    .v_total = 524,
    .v_sync_polarity = 1,
    .enable_clock = 0,
    .clock_polarity = 0,
    .enable_den = 0
};

scanvideo_mode_t custom_mode = {
    .default_timing = &custom_timing,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 1,
    .yscale = 1,
};

// Forward declarations, so structure definition can be placed at the top of this file
static void common_process_line(genlock_t *genlock, int line);
static void off_init(genlock_t *genlock);
static void off_destroy(genlock_t *genlock);
static void vary_clk_init(genlock_t *genlock);
static void vary_clk_update(genlock_t *genlock, int value);
static void vary_clk_destroy(genlock_t *genlock);
#if USE_SCANVIDEO_PRIVATE
static void vary_htotal_init(genlock_t *genlock);
static void vary_htotal_post_init(genlock_t *genlock);
static void vary_htotal_update(genlock_t *genlock, int value);
static void vary_htotal_destroy(genlock_t *genlock);
static void vary_vtotal_init(genlock_t *genlock);
static void vary_vtotal_update(genlock_t *genlock, int value);
static void vary_vtotal_destroy(genlock_t *genlock);
#endif

static genlock_t genlock_implementations[] = {
    {
        .init               = off_init,
        .destroy            = off_destroy
    }
    ,
    {
        .vsync_target       =  14686, // Target for genlock vsync offset (in us)
        .min                =  0x4f8, // Min value
        .nominal            =  0x4ff, // Nominal value (for the PIO clock divider)
        .max                =  0x508, // Max value
        .coeff_a            = -0.016, // Controller: error derivative term
        .coeff_b            = -0.0016, // Controller: error term
        .init               = vary_clk_init,
        .process_line       = common_process_line,
        .update             = vary_clk_update,
        .destroy            = vary_clk_destroy
    }
#if USE_SCANVIDEO_PRIVATE
    ,
    {
        .vsync_target       =  14686, // Target for genlock vsync offset (in us)
        .min                =    -26, // Min value
        .nominal            =     -6, // Nominal value (for h_total)
        .max                =     14, // Max value
        .coeff_a            =  -0.10, // Controller: error derivative term
        .coeff_b            =  -0.01, // Controller: error term
        .init               = vary_htotal_init,
        .post_init          = vary_htotal_post_init,
        .process_line       = common_process_line,
        .update             = vary_htotal_update,
        .destroy            = vary_htotal_destroy
    }
    ,
    {
        .vsync_target       =  14702, // Target for genlock vsync offset (in us)
        .min                =    523, // Min value
        .nominal            =    524, // Nominal value (for v_total)
        .max                =    525, // Max value
        .coeff_a            = -0.00655, // Controller: error derivative term
        .coeff_b            = -0.000655, // Controller: error term
        .init               = vary_vtotal_init,
        .process_line       = common_process_line,
        .update             = vary_vtotal_update,
        .destroy            = vary_vtotal_destroy
    }
#endif
};

// ======================================================================
// Static global variables
// ======================================================================

static int original_clkdiv;

static int init_pending = 0;

static int debug = 0;

// ======================================================================
// Optional code that delves into the innards of scanvideo
// ======================================================================

#if USE_SCANVIDEO_PRIVATE

// Warning: genlock modes 2 (vary_htotal) and 3 (vary_vtotal) needs changes to
// pico-extras/src/rp2_common/pico_scanvideo_dpi/scanvideo.c to make some
// private data structures public.
//
// Line 193:
//     remove the static keyword, so struct timing_state is no longer private
//
// Line 183:
//     remove the static keyword, so video_htiming_load_offset is no longer private
//
//
// If you get a compile error here, then check you have made this change.

extern uint8_t video_htiming_load_offset;

typedef struct {
    int32_t v_active;
    int32_t v_total;
    int32_t v_pulse_start;
    int32_t v_pulse_end;
    // todo replace with plain polarity
    uint32_t vsync_bits_pulse;
    uint32_t vsync_bits_no_pulse;

    uint32_t a, a_vblank, b1, b2, c, c_vblank;
    uint32_t vsync_bits;
    uint16_t dma_state_index;
    int32_t timing_scanline;
} timing_state_t;

extern timing_state_t timing_state;

static timing_state_t saved_timing_state;

static void save_timing_state() {
    saved_timing_state = timing_state;
}

static void restore_timing_state() {
    timing_state = saved_timing_state;
}

#endif // USE_SCANVIDEO_PRIVATE

// ======================================================================
// Common Genlock Code
// ======================================================================

// Return the time between the Atom VSync and the VGA VSync in us
// (should be between 0 and ~16666)
static uint read_vsync_offset() {
    // This variable is static so the last result can be returned if the RX Fifo is empty
    static uint result = 0;
    if (!pio_sm_is_rx_fifo_empty(GENLOCK_PIO, GENLOCK_SM)) {
        uint32_t offset = 0;
        while (!pio_sm_is_rx_fifo_empty(GENLOCK_PIO, GENLOCK_SM)) {
           offset = pio_sm_get(GENLOCK_PIO, GENLOCK_SM);
        }
        result = (0xffffffff - offset) / (SYS_FREQ / KHZ / 2);
    }
    return result;
}

// Update genlock state on the new line
static void common_process_line(genlock_t *genlock, int line) {

    static int last_line = 0;
    static int last_error = 0;

    // Second phase of init
    if (init_pending && line == 1) {
        if (genlock->post_init) {
            genlock->post_init(genlock);
        }
        init_pending = 0;
    }

    // Calculate genlock correction on line 433 (at the end of the Atom active display)
    if (line == 433 && line != last_line) {

        // Read the current VSYNC offset (in us) and calculate difference from target
        int error = genlock->vsync_target - read_vsync_offset();

        // Optimize the direction for correction
        if (error < -9000) {
            error += 18000;
        } else if (error > 9000) {
            error -= 18000;
        }

        int previous  = (int) genlock->current;

        genlock->current += ((double) (error - last_error)) * genlock->coeff_a + ((double) error) * genlock->coeff_b;

        if (genlock->current < genlock->min) {
            genlock->current = genlock->min;
        } else if (genlock->current > genlock->max) {
            genlock->current = genlock->max;
        }

        int next = (int) genlock->current;
        if (next != previous) {
            genlock->update(genlock, next);
            if (debug) {
                printf("%4d %4d %4d\r\n", error, error - last_error, next);
            }
        }
        last_error = error;
    }
    last_line = line;
}

// ======================================================================
// Public interface
// ======================================================================

void genlock_initialize() {
    // Load PIO state machine to track the offset between VGA VS and Atom FS
    uint offset = pio_add_program(GENLOCK_PIO, &genlock_program);
    genlock_program_init(GENLOCK_PIO, GENLOCK_SM, offset);
    pio_sm_set_enabled(GENLOCK_PIO, GENLOCK_SM, true);
    // Read original "safe" value of clkdiv
    original_clkdiv = SCANVIDEO_PIO->sm[SCANVIDEO_SCANLINE_SM].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
}

void genlock_debug(int on) {
    debug = on;
}

genlock_t *genlock_factory(genlock_mode_t mode) {
    if (mode < GENLOCK_NUM_MODES) {
        return &genlock_implementations[mode];
    } else {
        return NULL;
    }
}

// ======================================================================
// Genlock Mode 0: Genlock off
// ======================================================================

void off_init(genlock_t *genlock) {
    if (debug) {
        printf("Genlock mode 0: Genlock off\r\n");
    }
}

void off_destroy(genlock_t *genlock) {
}

// ======================================================================
// Genlock Mode 1: Vary Clock Frequency
// ======================================================================

static inline void set_clkdiv(uint i) {
    pio_sm_set_clkdiv_int_frac(SCANVIDEO_PIO, SCANVIDEO_SCANLINE_SM, i >> 8, i & 0xff);
    pio_sm_set_clkdiv_int_frac(SCANVIDEO_PIO, SCANVIDEO_TIMING_SM,   i >> 8, i & 0xff);
    pio_clkdiv_restart_sm_mask(SCANVIDEO_PIO, (1<<SCANVIDEO_SCANLINE_SM | 1 << SCANVIDEO_TIMING_SM));
}

void vary_clk_init(genlock_t *genlock) {
    if (debug) {
        printf("Genlock mode 1: Genlock by varying clkdiv\r\n");
    }
    genlock->current = genlock->nominal;
}

void vary_clk_update(genlock_t *genlock, int value) {
    set_clkdiv(value);
}

void vary_clk_destroy(genlock_t *genlock) {
    set_clkdiv(original_clkdiv);
}

// ======================================================================
// Genlock Mode 2: Vary H Total (horizontal line length)
// ======================================================================

// This genlock mode forces the scanvideo timing state machine to run
// at the system clock (rather than system clock / 5). The line length
// is increased by 5x, from 800 to 4000, to compensate so the timing
// remains the same. This results a 250ppm (1/4000) adjustment step,
// rather than the 780ppm (1/1280) adjustment step with the fractional
// divider

#if USE_SCANVIDEO_PRIVATE

static uint32_t nominal_c;
static uint32_t nominal_c_vblank;

// The a, a_vblank, b1, b2, c, c_vblank are pixel counts for various
// parts of the horizontal line. Their effective values need
// multiplying by 5, as we want to clock the timing state machine at
// the system clock, rather than at the 2x the pixel clock. But there
// is a complication, as these values have been corrected for some
// fixed overheads in the timing.pio state machine. These are:
//
// out exec, 16        2
// out x, 13           1
// out pins, 3         1
// loop:
// nop                 1 (because x=1 would go around the loop twice)
// jmp x-- loop        1
//
// This is 6 cycles, which is equivalent to three pixels (at a 2x pixel clock)
//
// scanvideo.c has a #define for this correction factor, which we reproduce here:
#define TIMING_CYCLE 3u

// For reference, this is used in the scanvideo.c timing_encode() macro:
// #define timing_encode(state, length, pins) ((video_htiming_states_program.instructions[state] ^ side_set_xor)| (((uint32_t)(length) - TIMING_CYCLE) << 16u) | ((uint32_t)(pins) << 29u))
//
// From this, we can see the  dma state values are encoded as:
// bits 31..29 = sync values
// bits 28..16 = length
// bits 15.. 0 = instruction
//
// We need to patch the length, so the timing is the same with a 10x
// clock, taking account of this correction factor.
//   i.e. length := ((length + 3) * 10) - 6

// We have a second version that is used when calculating new values,
// as the overhead is different, because we have patched the
// video_htiming smc

#define TIMING_CYCLE2 6u

uint32_t patch_htiming(uint32_t value) {
    uint32_t length = ((value >> 16) & 0x1FFF) + TIMING_CYCLE;
    length *= 10;
    return (value & 0xE000FFFF) | (length - TIMING_CYCLE2) << 16;
}

void vary_htotal_init(genlock_t *genlock) {
    if (debug) {
        printf("Genlock mode 2: Genlock by varying htotal\r\n");
    }

    genlock->current = genlock->nominal;

    save_timing_state();

    // Patch all the htiming state variables to maintain the same
    // timing when we run scanvideo video_htiming running at 10x the
    // expected speed.

    timing_state.a        = patch_htiming(timing_state.a        );
    timing_state.a_vblank = patch_htiming(timing_state.a_vblank );
    timing_state.b1       = patch_htiming(timing_state.b1       );
    timing_state.b2       = patch_htiming(timing_state.b2       );
    timing_state.c        = patch_htiming(timing_state.c        );
    timing_state.c_vblank = patch_htiming(timing_state.c_vblank );

    nominal_c        = timing_state.c;
    nominal_c_vblank = timing_state.c_vblank;

    init_pending = 1;
}

static void vary_htotal_post_init(genlock_t *genlock) {

    // The remaining 2x comes for patching the video_htiming state
    // machine, so it takes one clock per pixel indead of two
    //
    // In video_htimimg change:
    //
    // loop: nop
    //       jmp x-- loop
    //
    // to
    //
    //       nop
    // loop: jmp x-- loop
    //
    SCANVIDEO_PIO->instr_mem[video_htiming_load_offset + 5] = pio_encode_jmp_x_dec(video_htiming_load_offset + 5);

    // 5x of the 10x increase comes from reducing the PIO clock
    // divider used by video_htiming state machine from 5 to 1
    pio_sm_set_clkdiv_int_frac(SCANVIDEO_PIO, SCANVIDEO_SCANLINE_SM, 4, 255);
    pio_sm_set_clkdiv_int_frac(SCANVIDEO_PIO, SCANVIDEO_TIMING_SM, 1, 0);
    pio_clkdiv_restart_sm_mask(SCANVIDEO_PIO, (1<<SCANVIDEO_SCANLINE_SM | 1 << SCANVIDEO_TIMING_SM));
}

void vary_htotal_update(genlock_t *genlock, int value) {
    if (value < 0) {
        timing_state.c        = nominal_c - (-value << 16);
        timing_state.c_vblank = nominal_c_vblank - (-value << 16);
    } else {
        timing_state.c        = nominal_c + (value << 16);
        timing_state.c_vblank = nominal_c_vblank + (value << 16);
    }
}

void vary_htotal_destroy(genlock_t *genlock) {
    restore_timing_state();
    set_clkdiv(original_clkdiv);
    SCANVIDEO_PIO->instr_mem[video_htiming_load_offset + 5] = pio_encode_jmp_x_dec(video_htiming_load_offset + 4);
}

#endif

// ======================================================================
// Genlock Mode 3: Vary V Total (number of horizontal lines)
// ======================================================================

#if USE_SCANVIDEO_PRIVATE

static int v_back_porch = 0;

void vary_vtotal_init(genlock_t *genlock) {
    if (debug) {
        printf("Genlock mode 3: Genlock by varying vtotal\r\n");
    }
    genlock->current = genlock->nominal;
    save_timing_state();
    v_back_porch = timing_state.v_total - timing_state.v_pulse_end;
}

void vary_vtotal_update(genlock_t *genlock, int value) {
    timing_state.v_total     = value;
    timing_state.v_pulse_end = value - v_back_porch;
}

void vary_vtotal_destroy(genlock_t *genlock) {
    restore_timing_state();
}

#endif
