
fc.MOTOR__MIN(1000);
fc.MOTOR__MAX(2000);

fc.CTRL__RESET_INT(1);
fc.CTRL__RESET_INT(0);

fc.PITCH_P(double(typecast(single(0.0),'uint32')));
fc.PITCH_I(double(typecast(single(0.1),'uint32')));
fc.PITCH_D(double(typecast(single(0.0),'uint32')));

fc.ROLL_P(double(typecast(single(0.0),'uint32')));
fc.ROLL_I(double(typecast(single(0.1),'uint32')));
fc.ROLL_D(double(typecast(single(0.0),'uint32')));

fc.YAW_P(double(typecast(single(0.0),'uint32')));
fc.YAW_I(double(typecast(single(0.1),'uint32')));
fc.YAW_D(double(typecast(single(0.0),'uint32')));