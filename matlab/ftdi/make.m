if strcmp(computer,'PCWIN64')
   mex ftdi.cpp -L. -lftd2xx64 
else
   mex ftdi.cpp -L. -lftd2xx 
end