restoredefaultpath
addpath reg
addpath utils
addpath ftdi

clear all

global fc
global mpu
global rf
global ser

fc = fc_reg;
mpu = mpu_reg;
rf = sx1276_reg;

delete(instrfindall);
ser = serial('COM11');
ser.BaudRate = 115200;
ser.Timeout = 1;
fopen(ser);

ftdi('close');
% ftdi('open',0);
% ftdi('set_MPSSE');
% ftdi('clock',1e6);