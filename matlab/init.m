restoredefaultpath
addpath reg
addpath utils

clear all

global fc
global mpu
global ser

fc = fc_reg;
mpu = mpu_reg;

delete(instrfindall);
ser = serial('COM18');
ser.Timeout = 1;
fopen(ser);