restoredefaultpath
addpath reg
addpath utils
try
	fclose(ser);
catch
end
clear all
global fc
global mpu
global ser
fc = fc_reg;
mpu = mpu_reg;
ser = serial('COM18');
ser.Timeout = 1;
fopen(ser);
