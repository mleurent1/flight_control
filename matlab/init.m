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
global mpu
fc = fc_reg;
mpu = mpu_reg;
ftdi('open',0);
ftdi('set_serial');