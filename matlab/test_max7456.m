global osd

osd.VM0__SOFT_RST(1);
while osd.STAT__RST_BUSY
   sleep(1);
end

osd.OSDBL__AUTO_OSDBL_DISABLE(0);
% osd.VM0__PAL_NOT_NTSC(osd.STAT__PAL_DETECTED);
osd.VM0__PAL_NOT_NTSC(1);
osd.VM0__OSD_EN(1);
% osd.HOS(40);
% osd.VOS(22);
osd.HOS(45);
osd.VOS(28);

%%
osd.DMM__CLR_DISPLAY_MEM(1);
sleep(1);

%%
for row = 1:16
   addr = (row-1)*30;
   osd.DMAH__DMA_8(bitget(addr,9));
   osd.DMAL(mod(addr,2^8));
   osd.DMM__AUTO_INCR_EN(1);
   for line = 1:16
      fwrite(ser,[1,(row-1)*16+line-1]);
      while ser.BytesAvailable < 1
         sleep(1);
      end
      fread(ser,1);
   end
   fwrite(ser,[1,255]);
   while ser.BytesAvailable < 1
      sleep(1);
   end
   fread(ser,1);
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
   fwrite(ser,[1,char_addr(n)]);
   while ser.BytesAvailable < 1
      sleep(1);
   end
   fread(ser,1);
end
fwrite(ser,[1,255]);
while ser.BytesAvailable < 1
   sleep(1);
end
fread(ser,1);

addr = 7*30 + 10;
osd.DMAH__DMA_8(bitget(addr,9));
osd.DMAL(mod(addr,2^8));
osd.DMM__AUTO_INCR_EN(1);

str = '4.185';
char_addr = str - '1' + 1;
char_addr(char_addr<0) = 65;

for n = 1:length(char_addr)
   fwrite(ser,[1,char_addr(n)]);
   while ser.BytesAvailable < 1
      sleep(1);
   end
   fread(ser,1);
end
fwrite(ser,[1,255]);
while ser.BytesAvailable < 1
   sleep(1);
end
fread(ser,1);

