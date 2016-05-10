function dout= spi(addr,din)
	if argn(2) == 1
		ftdi('write',[2,addr,0,0]);
		sleep(10)
		dout = ftdi('read');
	else
		ftdi('write',[3,addr,0,din]);
		dout = [];
	end
endfunction

