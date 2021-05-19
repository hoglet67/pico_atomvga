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
#include "atomvga.h"

// This base address of the 8255 PIA
#define PIA_ADDR 0xB000

// The base address of the FRame Buffer
#define FB_ADDR 0x8000

#define vga_mode vga_mode_320x240_60
// #define vga_mode vga_mode_640x480_60

const uint LED_PIN = 25;
const uint SEL1_PIN = test_PIN_SEL1;
const uint SEL2_PIN = test_PIN_SEL1 + 1;
const uint SEL3_PIN = test_PIN_SEL1 + 2;

static PIO pio = pio1;
static uint8_t *fontdata = fontdata_6847;

// Initialise the GPIO pins - overrides whatever the scanvideo library did
static void initialiseIO()
{
    // Grab the uart pins back from the video function
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // pins 2 to 9 are used to read the 6502 bus - 8 bits at a time
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

void core1_func();

static semaphore_t video_initted;
static bool invert;

bool updated;

volatile uint8_t memory[0x10000];

#define CSI "\x1b["

int get_mode()
{
    return (memory[PIA_ADDR] & 0xf0) >> 4;
}

bool alt_colour()
{
    return !!(memory[PIA_ADDR + 2] & 0x8);
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
            unsigned char c = memory[row * 32 + col + FB_ADDR];
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
            if (address = 0xBDE0)
            {
                pio_sm_put(pio, 1, count++);
            }
        }
        else
        {
            u_int8_t data = (reg & 0xFF0000) >> 16;
            memory[address] = data;
            if (address >= FB_ADDR && address < FB_ADDR + 0x200 || address == PIA_ADDR)
            {
                updated = true;
                gpio_put(LED_PIN, 1);
            }
        }
    }
}

const uint debug_test_len = 32;

char debug_text[32];

volatile bool debug = false;

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
        c = c + 0x20;
        if (c < 0x80)
        {
            c = c ^ 0x60;
        }
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

        sprintf(buffer, "mode %x/%d %dx%d %s %d",
                mode,
                m,
                get_width(mode),
                get_height(mode),
                is_colour(mode) ? "col" : "b&w",
                bytes);

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
            if (address == 0xBDE0)
            {
                uint8_t b = memory[0xBDE0];
                pio_sm_put(pio, 1, 0xFF | (b << 8));
            }
            else if (address == 0xBDEF)
            {
                uint8_t b = 0x12;
                pio_sm_put(pio, 1, 0xFF | (b << 8));
            }
        }
        else
        {
            // write
            u_int8_t data = (reg & 0xFF0000) >> 16;
            memory[address] = data;
        }
    }
}

int main(void)
{
    uint base_freq = 50000;

    set_sys_clock_khz(base_freq * 3, true);
    setup_default_uart();

    stdio_init_all();

    for (uint i = FB_ADDR; i < FB_ADDR + 0x200; i++)
    {
        memory[i] = rand();
    }

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

const uint vdu_mem_start = FB_ADDR;
const uint vdu_mem_end = FB_ADDR + 0x1800;
const uint chars_per_row = 32;

const uint vga_width = 320;
const uint vga_height = 240;

const uint max_width = 256;
const uint max_height = 192;

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

uint16_t *do_text(scanvideo_scanline_buffer_t *buffer, uint relative_line_num, char *memory, uint16_t *p)
{
    // Screen is 16 rows x 32 columns
    // Each char is 12 x 8 pixels
    uint row = relative_line_num / 12;
    uint sub_row = relative_line_num % 12;

    if (row >= 0 && row < 16)
    {
        uint vdu_address = chars_per_row * row;
        for (int col = 0; col < 32; col++)
        {
            uint ch = memory[vdu_address + col];
            uint colour_index = (ch >> 6) & 0b11;
            if (alt_colour())
            {
                colour_index += 4;
            }
            uint16_t colour = text_palette[colour_index];
            if (ch >= 0x40 && ch <= 0x7F || ch >= 0xC0 && ch <= 0xFF)
            {
                uint pix_row = 2 - (sub_row / 4);
                uint16_t pix0 = ((ch >> (pix_row * 2)) & 0x1) ? colour : 0;
                uint16_t pix1 = ((ch >> (pix_row * 2)) & 0x2) ? colour : 0;
                *p++ = COMPOSABLE_COLOR_RUN;
                *p++ = pix1;
                *p++ = 4 - 3;
                *p++ = COMPOSABLE_COLOR_RUN;
                *p++ = pix0;
                *p++ = 4 - 3;
            }
            else
            {
                // uint colour_index = (ch >> 6) & 0b11;
                // if (alt_colour()) {
                //     colour_index += 4;
                // }
                uint16_t colour = text_palette[colour_index];
                if (alt_colour)
                    colour_index += 4;

                uint8_t b = fontdata[(ch & 0x3f) * 12 + sub_row];

                if (support_lower && ch >= 0x80 && ch < 0xA0)
                {
                    b = fontdata[((ch & 0x3f) + 64) * 12 + sub_row];
                }
                else if (ch >= 0x80)
                {
                    b = ~b;
                }
                uint8_t mask = 0x80;
                *p++ = COMPOSABLE_RAW_RUN;
                *p++ = (b & mask) ? colour : 0;
                *p++ = 8 - 3;
                for (mask = mask >> 1; mask > 0; mask = mask >> 1)
                {
                    *p++ = (b & mask) ? colour : 0;
                }
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
    uint debug_end = debug ? debug_start + 12 : debug_start;

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
            p = do_text(buffer, line_num - debug_start, debug_text, p);
        }
        else if (!(mode & 1))
        {
            if (relative_line_num >= 0 && relative_line_num < (16 * 12))
            {
                p = do_text(buffer, relative_line_num, (char *)memory + vdu_mem_start, p);
            }
        }
        else
        {

            const uint height = get_height(mode);
            relative_line_num = relative_line_num * height / 192;
            if (relative_line_num >= 0 && relative_line_num < height)
            {

                uint vdu_address = vdu_mem_start + bytes_per_row(mode) * relative_line_num;
                uint32_t *bp = (uint32_t *)memory + vdu_address / 4;

                *p++ = COMPOSABLE_RAW_RUN;
                *p++ = border_colour;
                *p++ = 256 + 1 - 3;

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
                        if (pixel_count == 256)
                        {
                            *p++ = colour;
                        }
                        else if (pixel_count == 128)
                        {
                            *p++ = colour;
                            *p++ = colour;
                        }
                        else if (pixel_count == 64)
                        {
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
                    for (uint i = 0; i < pixel_count / 32; i++)
                    {
                        uint32_t b = __builtin_bswap32(*bp++);
                        for (uint32_t mask = 0x80000000; mask > 0; mask = mask >> 1)
                        {
                            uint16_t colour = (b & mask) ? palette[0] : 0;
                            if (pixel_count == 256)
                            {
                                *p++ = colour;
                            }
                            else if (pixel_count == 128)
                            {
                                *p++ = colour;
                                *p++ = colour;
                            }
                            else if (pixel_count == 64)
                            {
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

void core1_func()
{
    // initialize video and interrupts on core 1
    scanvideo_setup(&vga_mode);
    initialiseIO();
    scanvideo_timing_enable(true);
    sem_release(&video_initted);
    while (true)
    {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        draw_color_bar(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}
