function save_config

global fc
global ser

fwrite(ser,[6,0,0,0,0,0]);
sleep(2000)

for n = 1:length(fc.flash_addr_list)
	fwrite(ser,[0,fc.flash_addr_list(n),0,0,0,0]);
	r = fread(ser,4);
	fwrite(ser,[5,fc.flash_addr_list(n),r']);
end

end