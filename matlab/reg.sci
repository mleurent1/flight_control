function dout = reg(addr,din)
	if argn(2) == 1
		ftdi('write',[0,addr,0,0]);
		sleep(10)
		r = ftdi('read');
		dout = 2^8 * r(1) + r(2);
	else
		ftdi('write',[1,addr,floor(din/2^8),modulo(din,2^8)]);
		dout = [];
	end
endfunction
