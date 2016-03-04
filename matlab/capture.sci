function capture(mux,t)
	
	global reg
	
	n = 4096;
	c = zeros(1,n);
	
	figure(1);
	clf()
	plot2d([],rect=[1,-2^15,n,2^15])
	xpoly([],[]);
	l = gce();
	l.foreground = 2;

	rw(reg.CAPTURE_MUX,mux);
	
	tic;
	
	while toc() < t
		sleep(100);
		
		a = rw(reg.CAPTURE_AVAILABLE)
		rw(reg.SPI_ADDR_AUTO_INC_EN,0);
		r = spi_read(reg.CAPTURE_READ(1), a);
		rw(reg.SPI_ADDR_AUTO_INC_EN,1);
		r = r(1:2:length(r)) + 2^8*r(2:2:length(r));
		r(r>=2^15) = r(r>=2^15) - 2^16;
		
		c(1:n-length(r)) = c(length(r)+1:n);
		c(n-length(r)+1:n) = r;
		l.data = [(1:n)',c'];
	end
	
endfunction
