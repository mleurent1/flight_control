// Reset FPGA
rw(reg.FPGA_RESET,1);
sleep(1);

// FPGA version
v = rw(reg.VERSION);
mprintf('FPGA version: %d\n',v);

rw(reg.LED_MUX,1);
rw(reg.DBG_MUX,1);

// Reset MPU
i2c_write(107,128); 
sleep(1);

// MPU version
v = i2c_read(117,1);
mprintf('MPU version: %d\n',v);

i2c_write(107,0); // get MPU out of sleep
sleep(1);
i2c_write(26,1); // Filter ON (=> Fs=1kHz)
i2c_write(107,1); // clk = gyro
i2c_write(56,1); // enable interrupt

// Give I2C control to FPGA
rw(reg.I2C_HOST_CTRL,0);



