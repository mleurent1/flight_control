function config_mismatch

global fc
global ser

for n = 1:length(fc.flash_addr_list)
	srl_write(ser,sprintf("%s",[0,fc.flash_addr_list(n),0,0,0,0]));
	r1 = sum(double(srl_read(ser,4)) .* 2.^(24:-8:0));
	srl_write(ser,sprintf("%s",[4,fc.flash_addr_list(n),0,0,0,0]));
	r2 = sum(double(srl_read(ser,4)) .* 2.^(24:-8:0));
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