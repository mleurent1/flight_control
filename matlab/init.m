restoredefaultpath
addpath reg
addpath ftdi
addpath utils
try
	ftdi('close');
catch
end
clear all
global fc
global mpu
global TIMEOUT
fc = fc_reg;
mpu = mpu_reg;
ftdi('open',0);
ftdi('set_serial');
TIMEOUT = 1;