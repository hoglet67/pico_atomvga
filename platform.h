/*
  platform.h Defines dependent on target platform.
*/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#define PLATFORM_ATOM   1
#define PLATFORM_DRAGON 2

#ifndef PLATFORM
#define PLATFORM PLATFORM_DRAGON
#endif

// Maximum memory used by 6847
#define VID_MEM_SIZE    0x1800

#if (PLATFORM==PLATFORM_ATOM)
// This base address of the 8255 PIA
#define PIA_ADDR 0xB000

// The base address of the FRame Buffer
#define FB_ADDR 0x8000

// 80 Column mode control addresses
#define COL80_BASE  0xBDE0
#define COL80_FG    0xBDE4
#define COL80_BG    0xBDE5
#define COL80_STAT  0xBDEF
#define COL80_MASK  0xFFF0

// Macros to get VDU memory base
#define GetVidMemBase() FB_ADDR
#define GetVidMemEnd()  (FB_ADDR+VID_MEM_SIZE)

#define DEBUG_MESS  "ATOM PICO VGA ADAPTER"

// Masks to extract Inverse and !A/S bits from incoming video data in text / sg modes
#define INV_MASK    0x80
#define AS_MASK     0x40
#define INTEXT_MASK AS_MASK

#define GetIntExt(ch)   (ch & INTEXT_MASK) ? true : false

// Always 0 on Atom
#define GetSAMSG()      0

#define LOWER_START   0x80
#define LOWER_END     0xA0
#define LOWER_RANGE   (LOWER_END-LOWER_START)
#define LOWER_INVERT  0

#define CMD_BASE	0xF000

#define ATOM_EOL	0x0D
#define SPACE	    0x20

#define CMD_LEN	16
#define CMD_FONT	"FONT"

#define ClearCommand()	memory[CMD_BASE]=0x00

// 6502 reset vector
#define RESET_VEC 0xFFFC

#define VDG_SPACE 32
#endif

#if (PLATFORM==PLATFORM_DRAGON)
// This base address of the 6821 PIA controlling the video
#define PIA_ADDR 0xFF22

// The base address of the FRame Buffer, this is updated by SAM.
#define FB_ADDR 0x0400

// 80 Column mode control addresses
#define COL80_BASE  0xFF88
#define COL80_FG    0xFF89
#define COL80_BG    0xFF8A
#define COL80_STAT  0xFF8B
#define COL80_MASK  0xFFF8

volatile uint16_t    SAMBits;

#define SAM_BASE        0xFFC0
#define SAM_END         0xFFDF

// Mask to apply to SAM's address to select which bit to process.
#define SAM_ADDR_MASK   0x003E
#define SAM_ADDR_SHIFT  1

// Mask to apply to SAM's address to extract the data bit
#define SAM_DATA_MASK   0x0001

#define GetSAMBit(addr)         ((addr & SAM_ADDR_MASK) >> SAM_ADDR_SHIFT)
#define GetSAMData(addr)        (addr & SAM_DATA_MASK)
#define GetSAMDataMask(addr)    (1 << GetSAMBit(addr))

#define SAM_MODE_MASK   0x0007
#define SAM_VADDR_MASK  0x03F8
#define SAM_VADDR_SHIFT 6

#define SAM_SG_MASK     0x0006
#define SAM_SG_SHIFT    1

// Macros to get VDU memory base
#define GetVidMemBase() ((SAMBits & SAM_VADDR_MASK) << SAM_VADDR_SHIFT)
#define GetVidMemEnd()  (GetVidMemBase()+VID_MEM_SIZE)

//Get SAM video mode, used to decode semigraphics modes.
#define GetSAMMode()    (SAMBits & SAM_MODE_MASK)

#define GetSAMSG()      ((SAMBits & SAM_SG_MASK) >> SAM_SG_SHIFT)

#define DEBUG_MESS  "DRAGON PICO VGA ADAPTER"

// Masks to extract Inverse and !A/S bits from incoming video data in text / sg modes
#define INV_MASK    0x40
#define AS_MASK     0x80
#define INTEXT_MASK 0x10

#define GetIntExt(ch)   (memory[PIA_ADDR] & INTEXT_MASK) ? true : false;

#define DRAGON_CMD_ADDR     0xFF80
#define DRAGON_FONTNO_ADDR  0xFF81
#define DRAGON_INK_ADDR     0xFF82
#define DRAGON_PAPER_ADDR   0xFF83
#define DRAGON_INKALT_ADDR  0xFF84

#define DRAGON_CMD_NONE     0x00
#define DRAGON_CMD_DEBUG    0x01
#define DRAGON_CMD_NODEBUG  0x02
#define DRAGON_CMD_LOWER    0x03
#define DRAGON_CMD_NOLOWER  0x04
#define DRAGON_CMD_ARTIOFF  0x05
#define DRAGON_CMD_ARTI1    0x06
#define DRAGON_CMD_ARTI2    0x07
#define DRAGON_CMD_SAVEEE   0x08
#define DRAGON_CMD_LOADEE   0x09
#define DRAGON_CMD_AUTOOFF  0x0A
#define DRAGON_CMD_AUTOON   0x0B

#define LOWER_START   0x00
#define LOWER_END     0x40
#define LOWER_RANGE   (LOWER_END-LOWER_START)
#define LOWER_INVERT  1

// 6809 reset vector
#define RESET_VEC 0xFFFE

// EEprom offsets
// Note, ink, paper and ink_alt are *WORDS*
#define EE_AUTOLOAD   0x00
#define EE_FONTNO     0x01
#define EE_INK        0x02
#define EE_PAPER      0x04
#define EE_INK_ALT    0x08
#define EE_ISLOWER    0x0A

#define AUTO_ON       0x01
#define AUTO_OFF      0x00

#define VDG_SPACE     96
#endif


#endif
