n = 0;
addr = -1;
clear reg

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'VM0';
reg(n).subf{1} = {'BUF_ENB',0,0};
reg(n).subf{2} = {'SOFT_RST',1,1};
reg(n).subf{3} = {'OSD_VSYNC_EN',2,2};
reg(n).subf{4} = {'OSD_EN',3,3};
reg(n).subf{5} = {'SYNC_SEL_MODE',5,4};
reg(n).subf{6} = {'PAL_NOT_NTSC',6,6};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'VM1';
reg(n).subf{1} = {'BLINK_DUTY_CYCLE',1,0};
reg(n).subf{2} = {'BLINK_TIME',3,2};
reg(n).subf{3} = {'BACKGND_BRIGTHNESS',6,4};
reg(n).subf{4} = {'BACKGND_MODE',7,7};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'HOS';
reg(n).subf{1} = {'HOS',5,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'VOS';
reg(n).subf{1} = {'VOS',4,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'DMM';
reg(n).subf{1} = {'AUTO_INCR_EN',0,0};
reg(n).subf{2} = {'VSYNC_CLR',1,1};
reg(n).subf{3} = {'CLR_DISPLAY_MEM',2,2};
reg(n).subf{4} = {'INV',3,3};
reg(n).subf{5} = {'BLK',4,4};
reg(n).subf{6} = {'LBC',5,5};
reg(n).subf{7} = {'16_BITS_MODE',6,6};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'DMAH';
reg(n).subf{1} = {'DMA_8',0,0};
reg(n).subf{2} = {'BYTE_SEL',1,1};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'DMAL';
reg(n).subf{1} = {'DMA_7_0',7,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'DMDI';
reg(n).subf{1} = {'DMDI',7,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'CMM';
reg(n).subf{1} = {'CMM',7,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'CMAH';
reg(n).subf{1} = {'CMAH',7,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'CMAL';
reg(n).subf{1} = {'CMAL',5,0};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'CDMI';
reg(n).subf{1} = {'RIGHT_PIX',1,0};
reg(n).subf{2} = {'CENTER_RIGHT_PIX',3,2};
reg(n).subf{3} = {'CENTER_LEFT_PIX',5,4};
reg(n).subf{4} = {'LEFT_PIX',7,6};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'OSDM';
reg(n).subf{1} = {'OSD_MUX_SWITCH_TIME',2,0};
reg(n).subf{2} = {'OSD_RISE_FALL_TIME',5,4};

%%
addr = 15;

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB0';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB1';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB2';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB3';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB4';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB5';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB6';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB7';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB8';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB9';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB10';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB11';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB12';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB13';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB14';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'RB15';
reg(n).subf{1} = {'WHITE_LVL',1,0};
reg(n).subf{2} = {'BLACK_LVL',3,2};

%%
n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'STAT';
reg(n).subf{1} = {'PAL_DETECTED',0,0};
reg(n).subf{2} = {'NTSC_DETECTED',1,1};
reg(n).subf{3} = {'LOS',2,2};
reg(n).subf{4} = {'HSYNC_LVL',3,3};
reg(n).subf{5} = {'VSYNC_LVL',4,4};
reg(n).subf{6} = {'CHAR_MEM_BUSY',5,5};
reg(n).subf{7} = {'RST_BUSY',6,6};

addr = 47;

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'DMDO';
reg(n).subf{1} = {'DMDO',7,0};

addr = 63;

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'CMDO';
reg(n).subf{1} = {'RIGHT_PIX',1,0};
reg(n).subf{2} = {'CENTER_RIGHT_PIX',3,2};
reg(n).subf{3} = {'CENTER_LEFT_PIX',5,4};
reg(n).subf{4} = {'LEFT_PIX',7,6};

%%
addr = 107;

n = n + 1;
addr = addr + 1;
reg(n).addr = addr;
reg(n).name = 'OSDBL';
reg(n).subf{1} = {'DO_NOT_CHANGE',3,0};
reg(n).subf{2} = {'AUTO_OSDBL_DISABLE',4,4};
