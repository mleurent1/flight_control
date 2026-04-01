global fc
global osd

fc.CTRL__OSD_HOST_CTRL(1);
osd.VM0__OSD_EN(0);

ascii = 1;

f = fopen('max7456_default_char_mem.mcm','r');
already_written = [];

for m = 0:255

   fprintf('CHAR %d\n',m);

	if ascii
		if (m == 0) % space
			m1 = 32;
		elseif (m >= 1) && (m <= 9) % 1..9
			m1 = m-1+49;
		elseif m == 10 % 0
			m1 = 48;
		elseif (m >= 11) && (m <= 36) % A..Z
			m1 = m-11+65;
		elseif (m >= 37) && (m <= 62) % a..z
			m1 = m-37+97;
		elseif m == 63 % (
			m1 = 40;
		elseif m == 64 % )
			m1 = 41;
		elseif m == 65 % .
			m1 = 46;
		elseif m == 66 % ?
			m1 = 63;
		elseif m == 67 % ;
			m1 = 59;
		elseif m == 68 % :
			m1 = 58;
		elseif m == 69 % ,
			m1 = 44;
		elseif m == 70 % '
			m1 = 39;
		elseif m == 71 % /
			m1 = 47;
		elseif m == 72 % "
			m1 = 34;
		elseif m == 73 % -
			m1 = 45;
		elseif m == 74 % <
			m1 = 60;
		elseif m == 75 % >
			m1 = 62;
		elseif m == 76 % @
			m1 = 64;
		else
			m1 = m;
		end
	else
		m1 = m;
	end
	osd.CMAH(m1);
	
	for n = 1:64
      line = fgetl(f);
		if ~ischar(line)
         break;
		end
		osd.CMAL(n-1);
		osd.CDMI(bin2dec(line));
	end

	if ischar(line) && ~ismember(m1,already_written)
		osd.CMM(bin2dec('10100000'));
		while osd.STAT__CHAR_MEM_BUSY
			sleep(1);
		end
	end
	
	already_written = [already_written, m1];
end

fclose(f);