function i2c_write(addr,data)
	rw(reg.I2C_ADDR,addr);
	rw(reg.I2C_DATA_W,data);
	rw(reg.I2C_WEN,1);
	while rw(reg.I2C_BUSY)
		sleep(1);
	end
	if rw(reg.I2C_NACK)
		error('No acknowledge from I2C');
	end
	rw(reg.I2C_WEN,0);
endfunction
