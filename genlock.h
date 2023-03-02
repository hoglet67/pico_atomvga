#ifndef INCLUDE_GENLOCK_H
#define INCLUDE_GENLOCK_H

// With an Atom line of 63.695us (Nomimal Atom)
// 251.000000 (refdiv = 2; fbdiv = 251, vco = 1506 pd1 = 6, pd2 = 1, clkdiv = 0x4ff (1279), ppm =   3.44
#define SYS_FREQ 251000

// If REFDIV is other than 1, then specify other parameters for pll_init()
#define REFDIV 2
#define VCO 1506000000
#define PD1 6
#define PD2 1

#define SCANVIDEO_PIO         pio0
#define SCANVIDEO_SCANLINE_SM 0
#define SCANVIDEO_TIMING_SM   3

#define GENLOCK_PIO           pio0
#define GENLOCK_SM            1

#ifndef USE_SCANVIDEO_PRIVATE
#define USE_SCANVIDEO_PRIVATE 1
#endif

typedef enum {
    GENLOCK_OFF,
    GENLOCK_VARY_CLK,
#if USE_SCANVIDEO_PRIVATE
    GENLOCK_VARY_HTOTAL,
    GENLOCK_VARY_VTOTAL,
#endif
    GENLOCK_NUM_MODES
} genlock_mode_t;

typedef struct genlock {
    // Target and parameters
    int nominal;
    int vsync_target;
    int coarse_threshold;
    int coarse_delta;
    int fine_delta;
    int unlocked_threshold;
    int locked_threshold;
    // Functions
    void (*init)(struct genlock *);
    void (*post_init)(struct genlock *);
    void (*process_line)(struct genlock *, int line);
    void (*apply_correction)(struct genlock *, int correction);
    void (*destroy)(struct genlock *);
    // Variables
    int current;
} genlock_t;

extern scanvideo_mode_t custom_mode;

void genlock_initialize();

genlock_t *genlock_factory(genlock_mode_t mode);

#endif
