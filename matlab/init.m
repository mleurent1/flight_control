restoredefaultpath
addpath reg
addpath ftdi
addpath utils
try
	ftdi('close');
catch
end
clear
global fc
fc = fc_reg;
ftdi('open',0);
ftdi('set_serial');