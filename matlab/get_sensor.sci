function gx = get_sensor(reg_info,n)
	r = spi_read(reg_info(1),n*2);
	gx = r(1:2:n*2) + 2^8*r(2:2:n*2);
	gx(gx>=2^15) = gx(gx>=2^15) - 2^16;
endfunction
