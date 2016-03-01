funcprot(0);
getd
try
	ftdi('close');
catch
end
exec ftdi/loader.sce;
load reg.dat
ftdi('open',0);
ftdi('set_MPSSE');
ftdi('clock',15e6);
