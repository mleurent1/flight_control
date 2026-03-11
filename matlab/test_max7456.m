global osd

fc.CTRL__OSD_HOST_CTRL(1);

osd.VM0__SOFT_RST(1);
while osd.STAT__RST_BUSY
   sleep(1);
end

osd.OSDBL__AUTO_OSDBL_DISABLE(0);
% osd.VM0__PAL_NOT_NTSC(osd.STAT__PAL_DETECTED);
osd.VM0__PAL_NOT_NTSC(0);
osd.VM0__OSD_EN(1);
osd.HOS(40);
osd.VOS(25);

%%
osd.DMM__CLR_DISPLAY_MEM(1);

%% Display all characters
c = 0;
nb_lines = 28;
burst_mode = 1;

nb_rows = floor((256-c)/nb_lines);
rem = 256-c - nb_rows*nb_lines;

for row = 1:nb_rows
	addr = (row-1)*30;
	osd.DMAH__DMA_8(bitget(addr,9));
	osd.DMAL(mod(addr,2^8));
	osd.DMM__AUTO_INCR_EN(1);
	if ~burst_mode
		for character = 1:nb_lines
			osd.write([],c);
			c = c + 1;
		end
		osd.write([],255);
	else
		buf = zeros(1,2*nb_lines);
		buf(1:2:2*nb_lines) = c:c+nb_lines-1;
		osd.write([],[buf,255]);
		c = c + nb_lines;
	end
end
if rem
	addr = nb_rows*30;
	osd.DMAH__DMA_8(bitget(addr,9));
	osd.DMAL(mod(addr,2^8));
	osd.DMM__AUTO_INCR_EN(1);
	if ~burst_mode
		for character = 1:rem
			osd.write([],c);
			c = c + 1;
		end		
		osd.write([],255);
	else
		buf = zeros(1,2*rem);
		buf(1:2:end) = c:c+rem-1;
		osd.write([],[buf,255]);
		c = c + rem;
	end
end

%%
burst_mode = 1;

addr = 4*30 + 5;
osd.DMAH__DMA_8(bitget(addr,9));
osd.DMAL(mod(addr,2^8));
osd.DMM__AUTO_INCR_EN(1);
 
str = 'P PITCH AND ROLL';
char_addr = str - 'A' + 11;
char_addr(char_addr<0) = 0;

if ~burst_mode
	for n = 1:length(char_addr)
		osd.write([],char_addr(n));
	end
	osd.write([],255);
else
	buf = zeros(1,2*length(char_addr));
	buf(1:2:end) = char_addr;
	osd.write([],[buf,255]);
end

addr = 7*30 + 10;
osd.DMAH__DMA_8(bitget(addr,9));
osd.DMAL(mod(addr,2^8));
osd.DMM__AUTO_INCR_EN(1);

str = '4.185';
char_addr = str - '0';
char_addr(char_addr<0) = 65;

if ~burst_mode
	for n = 1:length(char_addr)
		osd.write([],char_addr(n));
	end
	osd.write([],255);
else
	buf = zeros(1,2*length(char_addr));
	buf(1:2:end) = char_addr;
	osd.write([],[buf,255]);
end

