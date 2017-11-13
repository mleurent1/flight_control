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
ser = serial('COM21');
ser.BaudRate = 115200;
ser.Timeout = 1;
fopen(ser);

% ftdi('close');
% ftdi('open',0);
% n = 1;
% while (ftdi('set_MPSSE') < 0) && (n <= 3)
%    n = n + 1;
% end
% ftdi('clock',1e6);
% 
% sx1272_start;