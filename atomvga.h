#ifndef ATOMVGA_H_
#define ATOMVGA_H_

// Changed scanvideo setup so colour bits are in LSB of word

#define RED_1 0x0001
#define RED_2 0x0002
#define RED (RED_2 | RED_1)

#define GREEN_1 0x0004
#define GREEN_2 0x0008
#define GREEN (GREEN_2 | GREEN_1)

#define BLUE_1 0x0010
#define BLUE_2 0x0020
#define BLUE (BLUE_2 | BLUE_1)

#define BLACK 0
#define YELLOW (RED | GREEN)
#define YELLOW_1 (RED_1 | GREEN_1)
#define WHITE (RED | GREEN | BLUE)
#define WHITE_1 (RED_1 | GREEN_1 | BLUE_1)
#define WHITE_2 (WHITE_1 << 1)
#define COLOUR YELLOW
#define CYAN (GREEN | BLUE)
#define ORANGE (RED | GREEN_2)
#define MAGENTA (RED | BLUE)

#define NO_COLOURS  9
uint16_t colour_palette_atom[NO_COLOURS] = {
    GREEN,
    YELLOW,
    BLUE,
    RED,
    WHITE,
    CYAN,
    MAGENTA,
    ORANGE,
    BLACK
};

uint16_t colour_palette_default[NO_COLOURS] = {
    GREEN,
    YELLOW,
    BLUE,
    RED,
    WHITE,
    CYAN,
    MAGENTA,
    ORANGE,
    BLACK
};

#define IDX_GREEN   0
#define IDX_YELLOW  1
#define IDX_BLUE    2
#define IDX_RED     3
#define IDX_WHITE   4
#define IDX_CYAN    5   
#define IDX_MAGENTA 6
#define IDX_ORANGE  7
#define IDX_BLACK   8

#define MAX_COLOUR  8

// This colour index resets palette mappings / boarder colours to defaults
#define IDX_DEFAULTS    255

#define DEF_INK     GREEN 
#define DEF_PAPER   BLACK
#define DEF_INK_ALT ORANGE


uint16_t colour_palette_vga80[8] = {
    BLACK,
    BLUE,
    GREEN,
    CYAN,
    RED,
    MAGENTA,
    YELLOW,
    WHITE
};

#define IDX80_GREEN   2
#define IDX80_YELLOW  6
#define IDX80_BLUE    1
#define IDX80_RED     4
#define IDX80_WHITE   7
#define IDX80_CYAN    3   
#define IDX80_MAGENTA 5
#define IDX80_ORANGE  IDX80_GREEN
#define IDX80_BLACK   0

// Ordered in vdg pallete order translates vdg->80 col pallete.
uint8_t vdgpal_to_80colpal [NO_COLOURS] = {
        IDX80_GREEN,
        IDX80_YELLOW,
        IDX80_BLUE,
        IDX80_RED,
        IDX80_WHITE,
        IDX80_CYAN,
        IDX80_MAGENTA,
        IDX80_ORANGE,
        IDX80_BLACK
}; 


#if 0
uint16_t colour_palette_improved[4] = {
    BLACK,
    YELLOW,
    GREEN,
    MAGENTA,
};
#endif 

uint16_t colour_palette_artifact1[4] = {
    BLACK,
    BLUE,
    ORANGE,
    WHITE,
};

uint16_t colour_palette_artifact2[4] = {
    BLACK,
    ORANGE,
    BLUE,
    WHITE,
};


// Masks to extract pixel colours from SG4 and SG6 bytes
#define SG4_COL_MASK    0x70
#define SG6_COL_MASK    0xC0

#define SG4_COL_SHIFT   4
#define SG6_COL_SHIFT   6

// Masks to extact bit paterns from SG4 and SG6
#define SG4_PAT_MASK    0x0F
#define SG6_PAT_MASK    0x3F

// Bytes / char array for text / semigraphics modes
// SG mode                     4  8  12 24  6
const uint sg_bytes_row[5]  = {1, 4, 6, 12, 1};

#define TEXT_INDEX  0
#define SG4_INDEX   0
#define SG8_INDEX   1
#define SG12_INDEX  2
#define SG24_INDEX  3
#define SG6_INDEX   4

uint16_t *colour_palette = colour_palette_atom;

//                             0  1a   1   2a    2   3a    3   4a    4
//                             0  1    2    3    4    5    6    7    8
const uint width_lookup[9] =  {32, 64, 128, 128, 128, 128, 128, 128, 256};
const uint height_lookup[9] = {16, 64,  64,  64,  96,  96, 192, 192, 192};

inline bool is_colour(uint mode);

uint get_width(uint mode)
{
    uint m = (mode + 1) / 2;
    return width_lookup[m];
};

uint get_height(uint mode)
{
    uint m = (mode + 1) / 2;
    return height_lookup[m];
};

uint bytes_per_row(uint mode)
{
    return is_colour(mode) ? get_width(mode) * 2 / 8 : get_width(mode) / 8;
};

#define COL80_OFF   0x00
#define COL80_ON    0x80
#define COL40_ON    0x40
#define COL80_ATTR  0x08


// Status bits
#define STATUS_NONE     0x00
#define STATUS_ARTI1    0x01
#define STATUS_ARTI2    0x02
#define STATUS_DEBUG    0x04
#define STATUS_LOWER    0x08
#define STATUS_AUTO     0x10
#define STATUS_80COL    0x20
#define STATUS_BORDER   0x40

#define STATUS_ARTI_MASK    0x03

#define get_artifact()  (status & STATUS_ARTI_MASK)


#endif /* ATOMVGA_H_ */
