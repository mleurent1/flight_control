function config_mismatch

global fc
global ser

for n = 1:length(fc.flash_addr_list)
	fwrite(ser,[0,fc.flash_addr_list(n),0,0,0,0]);
	r1 = sum(fread(ser,4) .* 2.^(0:8:24)');
	fwrite(ser,[4,fc.flash_addr_list(n),0,0,0,0]);
	r2 = sum(fread(ser,4) .* 2.^(0:8:24)');
	if r1 ~= r2
		if fc.flash_float_list(n)
			f1 = typecast(uint32(r1),'single');
			f2 = typecast(uint32(r2),'single');
			fprintf('%s = %f, %f (flash)\n', fc.flash_name_list{n}, f1, f2);
		else
			fprintf('%s = %d, %d (flash)\n', fc.flash_name_list{n}, r1, r2);
		end
	end
end

end