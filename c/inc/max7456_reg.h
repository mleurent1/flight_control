#define MAX7456_VM0 0

#define MAX7456_VM0__BUF_ENB (1 << 0)
#define MAX7456_VM0__SOFT_RST (1 << 1)
#define MAX7456_VM0__OSD_VSYNC_EN (1 << 2)
#define MAX7456_VM0__OSD_EN (1 << 3)
#define MAX7456_VM0__SYNC_SEL_MODE(x) (((x) & 0x03) << 4)
#define MAX7456_VM0__PAL_NOT_NTSC (1 << 6)

#define MAX7456_VM1 1

#define MAX7456_VM1__BLINK_DUTY_CYCLE(x) (((x) & 0x03) << 0)
#define MAX7456_VM1__BLINK_TIME(x) (((x) & 0x03) << 2)
#define MAX7456_VM1__BACKGND_BRIGTHNESS(x) (((x) & 0x07) << 4)
#define MAX7456_VM1__BACKGND_MODE (1 << 7)

#define MAX7456_HOS 2
#define MAX7456_VOS 3
#define MAX7456_DMM 4

#define MAX7456_DMM__AUTO_INCR_EN (1 << 0)
#define MAX7456_DMM__VSYNC_CLR (1 << 1)
#define MAX7456_DMM__CLR_DISPLAY_MEM (1 << 2)
#define MAX7456_DMM__INV (1 << 3)
#define MAX7456_DMM__BLK (1 << 4)
#define MAX7456_DMM__LBC (1 << 5)
#define MAX7456_DMM__16_BITS_MODE (1 << 6)

#define MAX7456_DMAH 5

#define MAX7456_DMAH__DMA_8 (1 << 0)
#define MAX7456_DMAH__BYTE_SEL (1 << 1)

#define MAX7456_DMAL 6
#define MAX7456_DMDI 7
#define MAX7456_CMM 8
#define MAX7456_CMAH 9
#define MAX7456_CMAL 10
#define MAX7456_CDMI 11

#define MAX7456_CDMI__RIGHT_PIX(x) (((x) & 0x03) << 0)
#define MAX7456_CDMI__CENTER_RIGHT_PIX(x) (((x) & 0x03) << 2)
#define MAX7456_CDMI__CENTER_LEFT_PIX(x) (((x) & 0x03) << 4)
#define MAX7456_CDMI__LEFT_PIX(x) (((x) & 0x03) << 6)

#define MAX7456_OSDM 12

#define MAX7456_OSDM__OSD_MUX_SWITCH_TIME(x) (((x) & 0x07) << 0)
#define MAX7456_OSDM__OSD_RISE_FALL_TIME(x) (((x) & 0x03) << 4)

#define MAX7456_RB0 16

#define MAX7456_RB0__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB0__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB1 17

#define MAX7456_RB1__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB1__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB2 18

#define MAX7456_RB2__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB2__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB3 19

#define MAX7456_RB3__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB3__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB4 20

#define MAX7456_RB4__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB4__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB5 21

#define MAX7456_RB5__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB5__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB6 22

#define MAX7456_RB6__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB6__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB7 23

#define MAX7456_RB7__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB7__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB8 24

#define MAX7456_RB8__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB8__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB9 25

#define MAX7456_RB9__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB9__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB10 26

#define MAX7456_RB10__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB10__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB11 27

#define MAX7456_RB11__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB11__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB12 28

#define MAX7456_RB12__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB12__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB13 29

#define MAX7456_RB13__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB13__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB14 30

#define MAX7456_RB14__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB14__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_RB15 31

#define MAX7456_RB15__WHITE_LVL(x) (((x) & 0x03) << 0)
#define MAX7456_RB15__BLACK_LVL(x) (((x) & 0x03) << 2)

#define MAX7456_STAT 32

#define MAX7456_STAT__PAL_DETECTED (1 << 0)
#define MAX7456_STAT__NTSC_DETECTED (1 << 1)
#define MAX7456_STAT__LOS (1 << 2)
#define MAX7456_STAT__HSYNC_LVL (1 << 3)
#define MAX7456_STAT__VSYNC_LVL (1 << 4)
#define MAX7456_STAT__CHAR_MEM_BUSY (1 << 5)
#define MAX7456_STAT__RST_BUSY (1 << 6)

#define MAX7456_DMDO 48
#define MAX7456_CMDO 64

#define MAX7456_CMDO__RIGHT_PIX(x) (((x) & 0x03) << 0)
#define MAX7456_CMDO__CENTER_RIGHT_PIX(x) (((x) & 0x03) << 2)
#define MAX7456_CMDO__CENTER_LEFT_PIX(x) (((x) & 0x03) << 4)
#define MAX7456_CMDO__LEFT_PIX(x) (((x) & 0x03) << 6)

#define MAX7456_OSDBL 108

#define MAX7456_OSDBL__DO_NOT_CHANGE(x) (((x) & 0x0F) << 0)
#define MAX7456_OSDBL__AUTO_OSDBL_DISABLE (1 << 4)

