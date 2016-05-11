clear;
funcprot(0);
getd
try
	ftdi('close');
catch
end
exec ftdi/loader.sce;
load reg.dat
global R
ftdi('open',0);
ftdi('set_serial');
