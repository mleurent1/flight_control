clear all

// Read excel file of register definition
[f,t,x,s] = xls_open('../hdl/reg.xls');
[v,ti] = xls_read(f,s(1));
mclose(f);

// Parse excel table
reg = []; // Matlab reg structure
b = 0; // bit cursor
rst = 0; // reset value per 8 bits
sw = ''; // string containing write registers declaration and assignement
sr0 = ''; // string containing read registers declaration
sr1 = ''; // string containing read registers assignement
sr2 = ''; // string containing read pulses assignement
addr0 = 0;
addr1 = 0;
first_nan = 0;
for n = 2:size(v,1)
	if ti(n,1)
		name = t(ti(n,1));
		len = v(n,2);
		dflt = v(n,3);
		wpulse = v(n,4);
		rpulse = v(n,5);
		
		if isnan(dflt) & ~first_nan
			b = 0;
			first_nan = 1;
			addr1 = addr0(size(addr0,2)) + 1;
		end
		
		addr = unique(floor([b:b+len-1]/8)); 
		bits = [b b+len-1] - addr(1)*8;
		mask = 2^(bits(2)+1) - 2^(bits(1));
		
		// update reg
		execstr(sprintf('reg.%s=[addr+addr1,mask,dflt];',name));
		
		// update rst
		if ~isnan(dflt)
			if min(addr) > max(addr0)
				rst(addr(1)+1) = 0;
			end
			if size(addr,2) > 1
				rst(addr(2:size(addr,2))+1) = 0;
			end
			len0 = len + bits(1);
			dflt0 = dflt * 2^bits(1);
			for m = 1:size(addr,2)
				i = min(8,len0);
				rst(addr(m)+1) = rst(addr(m)+1) + modulo(dflt0, 2^i);
				len0 = len0-i;
				dflt0 = floor(dflt0/2^i);
			end
		end
		
		// update sw
		if ~isnan(dflt)
			s0 = '';
			for m = 1:15-length(name)
				s0 = strcat([s0 ' ']);
			end
			if size(addr,2) == 1
				if len == 1
					sw = [sw msprintf('wire        %s_out%s =  regw[%d][%d];', name, s0, addr, bits(1))];
				else
					sw = [sw msprintf('wire [%2d:0] %s_out%s =  regw[%d][%d:%d];', len-1, name,s0, addr, bits(2), bits(1))];
				end
			else
				len0 = len;
				b0 = bits(1);
				s = '';
				for m = 1:size(addr,2)
					i = min(8-b0,len0);
					s(m) = msprintf('regw[%d][%d:%d]', addr(m), b0+i-1, b0);
					len0 = len0-i;
					b0 = 0;
				end
				s1 = msprintf('wire [%2d:0] %s_out%s = {', len-1, name, s0);
				for m = size(addr,2):-1:1
					s1 = strcat([s1 s(m)]);
					if m > 1
						s1 = strcat([s1 ', ']);
					end
				end
				s1 = strcat([s1 '};']);
				sw = [sw s1];
			end
		end
		
		// update sr0
		if isnan(dflt)
			if len == 1
				sr0 = [sr0 msprintf('wire        %s_in;', name)];
			else
				sr0 = [sr0 msprintf('wire [%2d:0] %s_in;', len-1, name)];
			end
			if ~isnan(rpulse)
				s0 = '';
				for m = 1:8-length(name)
					s0 = strcat([s0 ' ']);
				end
				sr2 = [sr2 msprintf('wire %s_rpulse%s= (spi_addr == 7''d%d) ? spi_ren : 1''b0;', name, s0, addr(size(addr,2))+addr1)]
			end
		end
		
		// update sr1
		if isnan(dflt)
			if size(addr,2) == 1
				if len == 1
					sr1 = [sr1 msprintf('assign regr[%2d][%d]   = %s_in;', addr, bits(1), name)];
					b2 = bits(1);
				else
					sr1 = [sr1 msprintf('assign regr[%2d][%d:%d] = %s_in;', addr, bits(2), bits(1), name)];
					b2 = bits(2);
				end
			else
				len0 = len;
				b0 = bits(1);
				b1 = 0;
				for m = 1:size(addr,2)
					i = min(8-b0,len0);
					sr1 = [sr1 msprintf('assign regr[%2d][%d:%d] = %s_in[%d:%d];', addr(m), b0+i-1, b0, name, b1+i-1, b1)];
					len0 = len0-i;
					b2 = b0+i-1; 
					b0 = 0;
					b1 = b1+i;
				end
			end
		end
		
		addr0 = addr;
		b = b + len;
	end
end

if isnan(dflt) & (b2<7)
	sr1 = [sr1 msprintf('assign regr[%d][7:%d] = 0;', addr0, b2+1)];
end

save reg.dat reg

nb_addr_w = size(rst,1);
nb_addr_r = max(addr)+1;

mprintf('\nNB_ADDR_W = %d\n',nb_addr_w)
mprintf('NB_ADDR_R = %d\n',nb_addr_r)

mprintf('\n//Write registers reset values\n')
for n = 1:nb_addr_w
	mprintf('regw[%d] <= %d;\n',n-1,rst(n));
end

mprintf('\n//Write registers declaration and assignement')
mprintf('%s\n',sw(:));

mprintf('\n//Read registers declaration')
mprintf('%s\n',sr0(:));

mprintf('\n//Read registers assignement')
mprintf('%s\n',sr1(:));

mprintf('\n//Read pulses assignement')
mprintf('%s\n',sr2(:));


