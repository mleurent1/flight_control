function data = spi_read(addr,bytes)
	r = ftdi('SPI',[bitand(addr,127),zeros(1,bytes)]);
	data = r(2:bytes+1);
endfunction
