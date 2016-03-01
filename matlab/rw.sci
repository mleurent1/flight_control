function dout = rw(reg_info,din)
	m = size(reg_info,2);
	addr = reg_info(1:m-2);
	mask = reg_info(m-1);
	if mask == 255
		if argn(2) == 1
			dout = spi_read(addr(1),1);
		else
			spi_write(addr(1),din);
			dout = [];
		end
	else
		for offset = 0:7
			if bitand(mask,2^offset)
				break;
			end
		end
		r = 0;
		for n = 1:size(addr,2)
			r = r + spi_read(addr(n),1) * 2^(8*(n-1));
		end
		if argn(2) == 1
			dout = bitand(r,mask)/2^offset;
		else
			maskb = 2^(8*size(addr,2))-1 - mask;
			w = bitand(din*2^offset,mask) + bitand(r,maskb);
			for n = 1:size(addr,2)
				spi_write(addr(n), modulo(floor(w/2^(8*(n-1))), 2^8));
			end
			dout = [];
		end
	end
endfunction
