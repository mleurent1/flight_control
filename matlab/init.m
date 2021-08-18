restoredefaultpath
addpath reg
addpath utils
% addpath ftdi

clear all

global fc
global mpu
global rf
global osd
global ser

fc = fc_reg;
mpu = mpu_reg;
rf = sx1276_reg;
osd = max7456_reg;

system('sudo chmod 777 /dev/ttyACM0');

delete(instrfindall);
ser = serial('/dev/ttyACM0');
ser.BaudRate = 115200;
ser.Timeout = 1;
ser.InputBufferSize = 8192;
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