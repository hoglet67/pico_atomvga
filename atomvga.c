/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "pico.h"
#include "atomvga.pio.h"
#include "atomvga_out.pio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"
#include "hardware/irq.h"
#include "hardware/vreg.h"
#include "atomvga.h"
#include "platform.h"

// PIA and frambuffer address moved into platform.h -- PHS


// #define vga_mode vga_mode_320x240_60
#define vga_mode vga_mode_640x480_60

const uint LED_PIN = 25;
const uint SEL1_PIN = test_PIN_SEL1;
const uint SEL2_PIN = test_PIN_SEL1 + 1;
const uint SEL3_PIN = test_PIN_SEL1 + 2;

static PIO pio = pio1;
static uint8_t *fontdata = fontdata_6847;

static uint32_t vga80_lut[128 * 4];

// Initialise the GPIO pins - overrides whatever the scanvideo library did
static void initialiseIO()
{
    // Grab the uart pins back from the video function
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // pins 2 to 9 are used to read the 6502 / 6809 bus - 8 bits at a time
    for (uint pin = 2; pin <= 9; pin++)
    {
        gpio_init(pin);
        gpio_set_dir(pin, false);
        gpio_set_function(pin, GPIO_FUNC_PIO1);
    }

    // Output enable for the 74lvc245 buffers
    gpio_pull_up(SEL1_PIN);
    gpio_pull_up(SEL2_PIN);
    gpio_pull_up(SEL3_PIN);

    gpio_set_function(SEL1_PIN, GPIO_FUNC_PIO1);
    gpio_set_function(SEL2_PIN, GPIO_FUNC_PIO1);
    gpio_set_function(SEL3_PIN, GPIO_FUNC_PIO1);

    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void reset_vga80();

void core1_func();

static semaphore_t video_initted;
static bool invert;

bool updated;

volatile uint8_t memory[0x10000];

#define CSI "\x1b["

// Returns video mode as far as VDG is concerned, with the bits :
//  b3  b2  b1  b0
//  GM2 GM1 GM0 A/G
int get_mode()
{
#if (PLATFORM == PLATFORM_ATOM)
    return (memory[PIA_ADDR] & 0xf0) >> 4;
#elif (PLATFORM == PLATFORM_DRAGON)
    return ((memory[PIA_ADDR] & 0x80) >> 7) | ((memory[PIA_ADDR] & 0x70) >> 3);
#endif
}

bool alt_colour()
{
#if (PLATFORM == PLATFORM_ATOM)
    return !!(memory[PIA_ADDR + 2] & 0x8);
#elif (PLATFORM == PLATFORM_DRAGON)
    return (memory[PIA_ADDR] & 0x08);
#endif
}

void pscreen()
{
    printf(CSI "H");
    printf(CSI "?25l");
    printf("+ - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - +\n");

    for (int row = 0; row < 16; row++)
    {
        printf("|");
        for (int col = 0; col < 32; col++)
        {
            unsigned char c = memory[row * 32 + col + GetVidMemBase()];
            if (c < 0x80)
            {
                c = c ^ 0x60;
            }
            c = c - 0x20;
            c = isprint(c) ? c : '.';
            printf(" %c", c);
        }
        printf(" |\n");
    }

    printf("+ - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - +\n");
    printf("MODE %x \n", get_mode());
}

static void irq_handler()
{
    static int count = 0;
    while (!pio_sm_is_rx_fifo_empty(pio, 0))
    {
        u_int32_t reg = pio_sm_get_blocking(pio, 0);
        u_int16_t address = reg & 0xFFFF;
        if (reg & 0x1000000)
        {
            if (address = COL80_BASE)
            {
                pio_sm_put(pio, 1, count++);
            }
        }
        else
        {
            u_int8_t data = (reg & 0xFF0000) >> 16;
            memory[address] = data;
            if (address >= GetVidMemBase() && address < GetVidMemBase() + 0x200 || address == PIA_ADDR)
            {
                updated = true;
                gpio_put(LED_PIN, 1);
            }
        }
    }
}

const uint debug_test_len = 32;

char debug_text[32];

bool debug = false;

void set_debug_text(char *text)
{
    strncpy(debug_text, text, debug_test_len);

    for (int i = strnlen(text, debug_test_len); i < debug_test_len; i++)
    {
        debug_text[i] = ' ';
    }

    for (int i = 0; i < debug_test_len; i++)
    {
        unsigned char c = debug_text[i];

#if (PLATFORM == PLATFORM_ATOM)
        c = c + 0x20;
        if (c < 0x80)
        {
            c = c ^ 0x60;
        }
#else
        if((c >= 0x20) && (c <= 0x3F))
            c=c+0x40;
        else if ((c >= 0x60) && (c <= 0x7F))
            c=c-0x60;
#endif
        debug_text[i] = c;
    }
}

void update_debug_text()
{
    if (debug)
    {
        char buffer[40];
        uint mode = get_mode();

        uint bytes = bytes_per_row(mode) * get_height(mode);

        uint m = (mode + 1) / 2;
#if (PLATFORM == PLATFORM_ATOM)
        sprintf(buffer, "mode %x/%d %dx%d %s %d",
                mode,
                m,
                get_width(mode),
                get_height(mode),
                is_colour(mode) ? "col" : "b&w",
                bytes);
#elif (PLATFORM == PLATFORM_DRAGON)
        sprintf(buffer, "mode %x/%d %dx%d %s %d %04X",
                mode,
                m,
                get_width(mode),
                get_height(mode),
                is_colour(mode) ? "col" : "b&w",
                bytes,
                SAMBits);
#endif
        set_debug_text(buffer);
    }
    else
    {
        set_debug_text("");
    }
}

bool is_command(char *cmd)
{
    char *p = (char *)memory + 0xf000;
    while (*cmd != 0)
    {
        if (*cmd++ != *p++)
        {
            return false;
        }
    }
    if (*p == 0xD)
    {
        *p = 0;
        return true;
    }
    return false;
}

volatile bool support_lower = false;

void __no_inline_not_in_flash_func(main_loop())
{
    while (true)
    {
        // Get event from SM 0
        u_int32_t reg = pio_sm_get_blocking(pio, 0);

        // Get the address
        u_int16_t address = reg & 0xFFFF;

        // Is it a read or write opertaion?
        if (reg & 0x1000000)
        {
            // read
            if (address == COL80_BASE)
            {
                uint8_t b = 0x12;
                pio_sm_put(pio, 1, 0xFF | (b << 8));
            }
            else if ((address & 0xFFF0) == COL80_BASE)
            {
                uint8_t b = memory[address];
                pio_sm_put(pio, 1, 0xFF | (b << 8));
            }
        }
        else
        {
            // write
            u_int8_t data = (reg & 0xFF0000) >> 16;
            memory[address] = data;
#if (PLATFORM == PLATFORM_DRAGON)
            // Update SAM bits when written to
            if ((address >= SAM_BASE) && (address <= SAM_END))
            {
                uint8_t     sam_data = GetSAMData(address);
                uint16_t    sam_mask = GetSAMDataMask(address);

                if (sam_data)
                    SAMBits |= sam_mask;
                else
                    SAMBits &= ~sam_mask;
            }
#endif
#if (PLATFORM == PLATFORM_ATOM)
            // hack to reset the vga80x40 mode when BREAK is pressed
            if (address == 0xb003 && data == 0x8a)
            {
                reset_vga80();
            }
#endif
        }
    }
}

int main(void)
{
    uint sys_freq = 200000;
    if (sys_freq > 250000)
    {
        vreg_set_voltage(VREG_VOLTAGE_1_25);
    }
    set_sys_clock_khz(sys_freq, true);
    setup_default_uart();

    stdio_init_all();

    for (uint i = GetVidMemBase(); i < GetVidMemBase() + 0x200; i++)
    {
        memory[i] = 32;
    }

    char mess[32];

    // Display message and build date/time
    set_debug_text(DEBUG_MESS);
    memcpy((char *)(memory + GetVidMemBase() + 0x0140), debug_text, 32);
    set_debug_text(__DATE__ " " __TIME__);
    memcpy((char *)(memory + GetVidMemBase() + 0x0160), debug_text, 32);
    snprintf(mess,32,"BASE=%04X, PIA=%04X, ",GetVidMemBase(),PIA_ADDR);
    set_debug_text(mess);
    memcpy((char *)(memory + GetVidMemBase()+0x180), debug_text, 32);

    // create a semaphore to be posted when video init is complete
    sem_init(&video_initted, 0, 1);

    // launch all the video on core 1, so it isn't affected by USB handling on core 0
    multicore_launch_core1(core1_func);

    // wait for initialization of video to be complete
    sem_acquire_blocking(&video_initted);

    //initialiseIO();

    uint offset = pio_add_program(pio, &test_program);
    test_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    offset = pio_add_program(pio, &atomvga_out_program);
    atomvga_out_program_init(pio, 1, offset);
    pio_sm_set_enabled(pio, 1, true);

    main_loop();
}

#if (PLATFORM == PLATFORM_ATOM)
void check_command()
{
    if (is_command("DEBUG"))
    {
        debug = true;
    }
    else if (is_command("NODEBUG"))
    {
        debug = false;
    }
    else if (is_command("LOWER"))
    {
        support_lower = true;
    }
    else if (is_command("NOLOWER"))
    {
        support_lower = false;
    }
    else if (is_command("CHARSET0"))
    {
        fontdata = fontdata_6847;
    }
    else if (is_command("CHARSET1"))
    {
        fontdata = fontdata_6847t1;
    }
    else if (is_command("CHARSET2"))
    {
        fontdata = fontdata_gime;
    }
}
#elif (PLATFORM == PLATFORM_DRAGON)
void check_command()
{
    uint8_t command = memory[DRAGON_CMD_ADDR];

    if (DRAGON_CMD_DEBUG == command)
    {
        debug = true;
    }
    else if (DRAGON_CMD_NODEBUG == command)
    {
        debug = false;
    }
    else if (DRAGON_CMD_LOWER == command)
    {
        support_lower = true;
    }
    else if (DRAGON_CMD_NOLOWER == command)
    {
        support_lower = false;
    }
    else if (DRAGON_CMD_CHAR0 == command)
    {
        fontdata = fontdata_6847;
    }
    else if (DRAGON_CMD_CHAR1 == command)
    {
        fontdata = fontdata_6847t1;
    }
    else if (DRAGON_CMD_CHAR2 == command)
    {
        fontdata = fontdata_gime;
    }
    memory[DRAGON_CMD_ADDR] = DRAGON_CMD_NONE;
}
#endif

const uint chars_per_row = 32;

const uint vga_width = 640;
const uint vga_height = 480;

const uint max_width = 512;
const uint max_height = 384;

const uint vertical_offset = (vga_height - max_height) / 2;
const uint horizontal_offset = (vga_width - max_width) / 2;

const uint debug_start = max_height + vertical_offset;

uint16_t *add_border(uint16_t *p, uint16_t border_colour, uint16_t len)
{
    *p++ = COMPOSABLE_COLOR_RUN;
    *p++ = border_colour;
    *p++ = len - 3;
    return p;
}

// Process Text mode, Semigraphics modes.
//
// 6847 control Atom    Dragon
// INV          D7      D6
// !A/G         PA4     PB7
// !A/S         D6      D7
// CSS          PC3     PB3
// !INT/EXT     D6      PB4
// GM0          PA5     PB4
// GM1          PA6     PB5
// GM2          PA7     PB6
//
// Due to the interaction between the SAM and the VDG on the Dragon, there are 3
// extra semigraphics modes, SG8, SG12 and SG24, that split the character space up
// into 8, 12 and 24 pixels. These pixels are still half a character width but are
// 3, 2 and 1 scanlines high respectively.
// This happens by programming the VDG in text mode and the SAM in graphics mode.
// The memory used for these modes is incresed so that each vertical part of the
// character space is 32 bytes apart in memory.
// For example in the SG8 mode, the top 2 pixels are encoded in the first byte
// the next 2 in the second byte and so on.
// In this way the bytes per character line are 128, 192, and 384.
//
// Mode     Bytes/line  Bytes/chars colours resolution  memory
// SG4      32          1           8       64x32       512
// SG6      32          1           4       64x48       512
// SG8      128         4           8       64x64       2048
// SG12     192         6           8       64x96       3072
// SG24     384         12          8       64x192      6144
//
// In SG8, SG12, SG24, the byte format is the same as the byte format for SG4
// However if ralative_line_no is < 6 pixels 2 and 3 are plotted. If it is > 6
// pixels 0 and 1 are plotted.
//

uint16_t *do_text(scanvideo_scanline_buffer_t *buffer, uint relative_line_num, char *memory, uint16_t *p, bool is_debug)
{
    // Screen is 16 rows x 32 columns
    // Each char is 12 x 8 pixels
    // Note we divide ralative_line_number by 2 as we are double scanning each 6847 line to
    // 2 VGA lines.
    uint row = (relative_line_num / 2) / 12;            // char row
    uint sub_row = (relative_line_num / 2) % 12;        // scanline within current char row
    uint sgidx = is_debug ? TEXT_INDEX : GetSAMSG();    // index into semigraphics table
    uint rows_per_char  = 12 / sg_bytes_row[sgidx];     // bytes per character space vertically

    if (row >= 0 && row < 16)
    {
        // Calc start address for this row
        uint vdu_address = ((chars_per_row * sg_bytes_row[sgidx]) * row) + (chars_per_row * (sub_row / rows_per_char));

        for (int col = 0; col < 32; col++)
        {
            // Get character data from RAM and extract inv,ag,int/ext
            uint ch = memory[vdu_address + col];
            bool inv    = (ch & INV_MASK) ? true : false;
            bool as     = (ch & AS_MASK) ? true : false;
            bool intext = GetIntExt(ch);

            if (as && intext)
                sgidx = SG6_INDEX;           // SG6

            uint colour_index;

            if (SG6_INDEX == sgidx)
                colour_index = (ch & SG6_COL_MASK) >> SG6_COL_SHIFT;
            else
                colour_index = (ch & SG4_COL_MASK) >> SG4_COL_SHIFT;

            if (alt_colour())
            {
                if (SG6_INDEX == sgidx)
                    colour_index += 4;
            }

            uint16_t colour = colour_palette_atom[colour_index];
            uint16_t back_colour = 0;

            // Deal with text mode first as we can decide this purely on the setting of the
            // alpha/semi bit.
            if(!as)
            {
                uint8_t b = fontdata[(ch & 0x3f) * 12 + sub_row];

                if (alt_colour())
                    colour = ORANGE;
                else
                    colour = GREEN;
                if (support_lower && ch >= LOWER_START && ch < LOWER_END)
                {
                    b = fontdata[((ch & 0x3f) + 64) * 12 + sub_row];
                }
                else if (inv)
                {
                    back_colour = colour;
                    colour = 0;
                }

                if (b == 0)
                {
                    *p++ = COMPOSABLE_COLOR_RUN;
                    *p++ = back_colour;
                    *p++ = 16 - 3;
                }
                else
                {
                    // bits 0,6 and 7 are always 0
                    *p++ = COMPOSABLE_RAW_RUN;
                    *p++ = back_colour;     // bit 7
                    *p++ = 16 - 3;
                    *p++ = back_colour;
                    *p++ = back_colour;     // bit 6
                    *p++ = back_colour;
                    for (uint8_t mask = 0x20; mask > 1; mask = mask >> 1)
                    {
                        const uint16_t c = (b & mask) ? colour : back_colour;
                        *p++ = c;
                        *p++ = c;
                    }
                    *p++ = back_colour;     // bit 0
                    *p++ = back_colour;
                }
            }
            else        // Semigraphics
            {
                uint pix_row = (SG6_INDEX == sgidx) ? 2 - (sub_row / 4) : 1 - (sub_row / 6);

                uint16_t pix0 = ((ch >> (pix_row * 2)) & 0x1) ? colour : back_colour;
                uint16_t pix1 = ((ch >> (pix_row * 2)) & 0x2) ? colour : back_colour;
                *p++ = COMPOSABLE_COLOR_RUN;
                *p++ = pix1;
                *p++ = 8 - 3;
                *p++ = COMPOSABLE_COLOR_RUN;
                *p++ = pix0;
                *p++ = 8 - 3;
            }
        }
    }
    return p;
}

void draw_color_bar(scanvideo_scanline_buffer_t *buffer)
{
    const uint mode = get_mode();
    const uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *)buffer->data;
    int relative_line_num = line_num - vertical_offset;

    if (line_num == 0)
    {
        check_command();
        update_debug_text();
    }

    uint16_t *palette = colour_palette;
    if (alt_colour())
    {
        palette += 4;
    }

    // Graphics modes have a coloured border, text modes have a black border
    uint16_t border_colour = (mode & 1) ? palette[0] : 0;
    uint debug_end = debug ? debug_start + 24 : debug_start;

    if (relative_line_num < 0 || line_num >= debug_end)
    {
        // Add top/bottom borders
        p = add_border(p, border_colour, vga_width);
    }
    else
    {

        // Add left border
        p = add_border(p, border_colour, horizontal_offset - 1);

        if (line_num >= debug_start && line_num < debug_end)
        {
            p = do_text(buffer, line_num - debug_start, debug_text, p, true);
        }
        else if (!(mode & 1))
        {
            if (relative_line_num >= 0 && relative_line_num < (16 * 24))
            {
                p = do_text(buffer, relative_line_num, (char *)memory + GetVidMemBase(), p, false);
            }
        }
        else
        {

            const uint height = get_height(mode);
            relative_line_num = (relative_line_num / 2) * height / 192;
            if (relative_line_num >= 0 && relative_line_num < height)
            {

                uint vdu_address = GetVidMemBase() + bytes_per_row(mode) * relative_line_num;
                uint32_t *bp = (uint32_t *)memory + vdu_address / 4;

                *p++ = COMPOSABLE_RAW_RUN;
                *p++ = border_colour;
                *p++ = 512 + 1 - 3;

                const uint pixel_count = get_width(mode);
                if (is_colour(mode))
                {
                    for (uint pixel = 0; pixel < pixel_count; pixel++)
                    {
                        uint32_t word;
                        if ((pixel % 16) == 0)
                        {
                            word = __builtin_bswap32(*bp++);
                        }
                        uint x = (word >> 30) & 0b11;
                        uint16_t colour = palette[x];
                        if (pixel_count == 128)
                        {
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                        }
                        else if (pixel_count == 64)
                        {
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                            *p++ = colour;
                        }
                        word = word << 2;
                    }
                }
                else
                {
                    uint16_t fg = palette[0];
                    for (uint i = 0; i < pixel_count / 32; i++)
                    {
                        const uint32_t b = __builtin_bswap32(*bp++);
                        if (pixel_count == 256)
                        {
                            for (uint32_t mask = 0x80000000; mask > 0;)
                            {
                                uint16_t colour = (b & mask) ? fg : 0;
                                *p++ = colour;
                                *p++ = colour;
                                mask = mask >> 1;

                                colour = (b & mask) ? fg : 0;
                                *p++ = colour;
                                *p++ = colour;
                                mask = mask >> 1;

                                colour = (b & mask) ? fg : 0;
                                *p++ = colour;
                                *p++ = colour;
                                mask = mask >> 1;

                                colour = (b & mask) ? fg : 0;
                                *p++ = colour;
                                *p++ = colour;
                                mask = mask >> 1;
                            }
                        }
                        else if (pixel_count == 128)
                        {
                            for (uint32_t mask = 0x80000000; mask > 0; mask = mask >> 1)
                            {
                                uint16_t colour = (b & mask) ? palette[0] : 0;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                            }
                        }
                        else if (pixel_count == 64)
                        {
                            for (uint32_t mask = 0x80000000; mask > 0; mask = mask >> 1)
                            {
                                uint16_t colour = (b & mask) ? palette[0] : 0;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                                *p++ = colour;
                            }
                        }
                    }
                }
            }
        }

        // Add right border
        p = add_border(p, border_colour, horizontal_offset);
    }

    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    if (!(3u & (uintptr_t)p))
    {
        *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    }
    else
    {
        *p++ = COMPOSABLE_EOL_ALIGN;
    }
    *p++ = 0;

    buffer->data_used = ((uint32_t *)p) - buffer->data;
    assert(buffer->data_used < buffer->data_max);

    buffer->status = SCANLINE_OK;
}

void reset_vga80()
{
    memory[COL80_BASE] = 0x00;  // Normal text mode (vga80 off)
    memory[COL80_FG] = 0xB2;    // Foreground Green
    memory[COL80_BG] = 0x00;    // Background Black
}

void initialize_vga80()
{
    // Reset the VGA80 hardware
    reset_vga80();
    // Use an LUT to allow two pixels to be calculated at once, taking account of the attribute byte for colours
    //
    // Bit  8  7  6  5  4  3  2  1   0
    //      --bgc--  x  --fgc--  p1 p0
    //
    for (int i = 0; i < 128 * 4; i++)
    {
        vga80_lut[i] = ((i & 1) ? colour_palette_vga80[(i >> 2) & 7] : colour_palette_vga80[(i >> 6) & 7]) << 16;
        vga80_lut[i] |= ((i & 2) ? colour_palette_vga80[(i >> 2) & 7] : colour_palette_vga80[(i >> 6) & 7]);
    }
}

uint16_t *do_text_vga80(scanvideo_scanline_buffer_t *buffer, uint relative_line_num, uint16_t *p)
{
    // Screen is 80 columns by 40 rows
    // Each char is 12 x 8 pixels
    uint row = relative_line_num / 12;
    uint sub_row = relative_line_num % 12;

    uint8_t *fd = fontdata + sub_row;

    if (row >= 0 && row < 40)
    {
        // Compute the start address of the current row in the Atom framebuffer
        volatile uint8_t *char_addr = memory + GetVidMemBase() + 80 * row;

        // Read the VGA80 control registers
        uint vga80_ctrl1 = memory[COL80_FG];
        uint vga80_ctrl2 = memory[COL80_BG];

        *p++ = COMPOSABLE_RAW_RUN;
        *p++ = 0;       // Extra black pixel
        *p++ = 642 - 3; //
        *p++ = 0;       // Extra black pixel

        // For efficiency, compute two pixels at a time using a lookup table
        // p is now on a word boundary due to the extra pixels above
        uint32_t *q = (uint32_t *)p;

        if (vga80_ctrl1 & 0x08)
        {
            // Attribute mode enabled, attributes follow the characters in the frame buffer
            volatile uint8_t *attr_addr = char_addr + 80 * 40;
            uint shift = (sub_row >> 1) & 0x06; // 0, 2 or 4
            // Compute these outside of the for loop for efficiency
            uint smask0 = 0x10 >> shift;
            uint smask1 = 0x20 >> shift;
            uint ulmask = (sub_row == 10) ? 0xFF : 0x00;
            for (int col = 0; col < 80; col++)
            {
                uint ch = *char_addr++;
                uint attr = *attr_addr++;
                uint32_t *vp = vga80_lut + ((attr & 0x77) << 2);
                if (attr & 0x80)
                {
                    // Semi Graphics
                    uint32_t p1 = (ch & smask1) ? *(vp + 3) : *vp;
                    uint32_t p0 = (ch & smask0) ? *(vp + 3) : *vp;
                    // Unroll the writing of the four pixel pairs
                    *q++ = p1;
                    *q++ = p1;
                    *q++ = p0;
                    *q++ = p0;
                }
                else
                {
                    // Text
                    uint8_t b = fd[(ch & 0x7f) * 12];
                    if (ch >= 0x80)
                    {
                        b = ~b;
                    }
                    // Underlined
                    if (attr & 0x08)
                    {
                        b |= ulmask;
                    }
                    // Unroll the writing of the four pixel pairs
                    *q++ = *(vp + ((b >> 6) & 3));
                    *q++ = *(vp + ((b >> 4) & 3));
                    *q++ = *(vp + ((b >> 2) & 3));
                    *q++ = *(vp + ((b >> 0) & 3));
                }
            }
        }
        else
        {
            // Attribute mode disabled, use default colours from the VGA80 control registers:
            //   bits 2..0 of VGA80_CTRL1 (#BDE4) are the default foreground colour
            //   bits 2..0 of VGA80_CTRL2 (#BDE5) are the default background colour
            uint attr = ((vga80_ctrl2 & 7) << 4) | (vga80_ctrl1 & 7);
            uint32_t *vp = vga80_lut + (attr << 2);
            for (int col = 0; col < 80; col++)
            {
                uint ch = *char_addr++;
                uint8_t b = fd[(ch & 0x7f) * 12];
                if (ch >= 0x80)
                {
                    b = ~b;
                }
                // Unroll the writing of the four pixel pairs
                *q++ = *(vp + ((b >> 6) & 3));
                *q++ = *(vp + ((b >> 4) & 3));
                *q++ = *(vp + ((b >> 2) & 3));
                *q++ = *(vp + ((b >> 0) & 3));
            }
        }
    }
    // The above loops add 80 x 4 = 320 32-bit words, which is 640 16-bit words
    return p + 640;
}

void draw_color_bar_vga80(scanvideo_scanline_buffer_t *buffer)
{
    const uint line_num = scanvideo_scanline_number(buffer->scanline_id);

    uint16_t *p = do_text_vga80(buffer, line_num, (uint16_t *)buffer->data);

    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    if (!(3u & (uintptr_t)p))
    {
        *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    }
    else
    {
        *p++ = COMPOSABLE_EOL_ALIGN;
    }
    *p++ = 0;

    buffer->data_used = ((uint32_t *)p) - buffer->data;
    assert(buffer->data_used < buffer->data_max);

    buffer->status = SCANLINE_OK;
}

void core1_func()
{
    // initialize video and interrupts on core 1
    initialize_vga80();
    scanvideo_setup(&vga_mode);
    initialiseIO();
    scanvideo_timing_enable(true);
    sem_release(&video_initted);
    while (true)
    {
        uint vga80 = memory[COL80_BASE] & 0x80;
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        if (vga80)
        {
            draw_color_bar_vga80(scanline_buffer);
        }
        else
        {
            draw_color_bar(scanline_buffer);
        }
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}
