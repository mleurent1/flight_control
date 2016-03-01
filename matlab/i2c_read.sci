function data = i2c_read(addr,bytes)
	rw(reg.I2C_ADDR,addr);
	rw(reg.I2C_READ_SIZE,bytes);
	rw(reg.I2C_REN,1);
	while rw(reg.I2C_BUSY)
		sleep(1);
	end
	if rw(reg.I2C_NACK)
		error('No acknowledge from I2C');
	end
	data = rw(reg.I2C_DATA_R);
	rw(reg.I2C_REN,0);
endfunction
