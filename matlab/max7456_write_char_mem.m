global osd

osd.VM0__OSD_EN(0);

f = fopen('max7456_default_char_mem.mcm','r');

for m = 1:256
   fprintf('CHAR %d\n',m);
   osd.CMAH(m-1);
   for n = 1:64
      line = fgetl(f);
      if ~ischar(line)
         break;
      end
      osd.CMAL(n-1);
      osd.CDMI(bin2dec(line));
   end
   if ~ischar(line)
      break;
   end
   osd.CMM(bin2dec('10100000'));
   while osd.STAT__CHAR_MEM_BUSY
      sleep(1);
   end
end

fclose(f);