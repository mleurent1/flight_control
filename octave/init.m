restoredefaultpath;
pkg load instrument-control
addpath reg
addpath utils

clear all

global fc
global mpu
global ser

fc = fc_reg;
mpu = mpu_reg;

ser = serial("COM9",115200,1);
%ser = serial("\\\\.\\COM12",115200,1);