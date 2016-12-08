function save_config

global fc
global ser

srl_write(ser,sprintf("%s",[6,0,0,0,0,0]));
sleep(1)

for n = 1:length(fc.flash_addr_list)
	srl_write(ser,sprintf("%s",[0,fc.flash_addr_list(n),0,0,0,0]));
	r = srl_read(ser,4);
	srl_write(ser,sprintf("%s",[5,fc.flash_addr_list(n),r]));
end

end