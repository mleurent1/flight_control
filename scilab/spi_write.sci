function spi_write(addr,data)
	ftdi('SPI',[128+bitand(addr,127),data]);
endfunction
