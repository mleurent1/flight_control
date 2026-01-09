global osd

osd.VM0__SOFT_RST(1);
while osd.STAT__RST_BUSY
   sleep(1);
end

osd.OSDBL__AUTO_OSDBL_DISABLE(0);
% osd.VM0__PAL_NOT_NTSC(osd.STAT__PAL_DETECTED);
osd.VM0__PAL_NOT_NTSC(0);
osd.VM0__OSD_EN(1);
osd.HOS(40);
osd.VOS(22);

%%
osd.DMM__CLR_DISPLAY_MEM(1);
sleep(1);

%% Display all characters
c = 0;
for row = 1:16
	addr = (row-1)*30;
	osd.DMAH__DMA_8(bitget(addr,9));
	osd.DMAL(mod(addr,2^8));
	osd.DMM__AUTO_INCR_EN(1);
	for character = 1:22
		c = c + 1;
		osd.write([],c-1);
	end
	osd.write([],255);
end

%%
addr = 4*30 + 5;
osd.DMAH__DMA_8(bitget(addr,9));
osd.DMAL(mod(addr,2^8));
osd.DMM__AUTO_INCR_EN(1);
 
str = 'P PITCH AND ROLL';
char_addr = str - 'A' + 11;
char_addr(char_addr<0) = 0;

for n = 1:length(char_addr)
	osd.write([],char_addr(n));
end
osd.write([],255);

addr = 7*30 + 10;
osd.DMAH__DMA_8(bitget(addr,9));
osd.DMAL(mod(addr,2^8));
osd.DMM__AUTO_INCR_EN(1);

str = '4.185';
char_addr = str - '0';
char_addr(char_addr<0) = 65;

for n = 1:length(char_addr)
	osd.write([],char_addr(n));
end
osd.write([],255);

