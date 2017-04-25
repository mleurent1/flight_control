n = 0;
clear reg

n = n + 1;
reg(n).name = 'VERSION';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'VERSION',7,0,'uint8',0};

n = n + 1;
reg(n).name = 'CTRL';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'MPU_HOST_CTRL',0,0,'uint8',0};
reg(n).subf{2} = {'RESET_INTEGRAL_ON_ARMED',1,1,'uint8',1};
reg(n).subf{3} = {'BEEP_TEST',2,2,'uint8',0};
reg(n).subf{4} = {'TIME_MAXHOLD',3,3,'uint8',0};
reg(n).subf{5} = {'LED_SELECT',5,4,'uint8',2};

n = n + 1;
reg(n).name = 'MOTOR_TEST';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'VALUE',15,0,'uint16',0};
reg(n).subf{2} = {'SELECT',19,16,'uint8',0};

n = n + 1;
reg(n).name = 'DEBUG';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'CASE',7,0,'uint8',0};
reg(n).subf{2} = {'MASK',23,8,'uint16',127};

n = n + 1;
reg(n).name = 'ERROR';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'MPU',15,0,'uint16',0};
reg(n).subf{2} = {'RADIO',31,16,'uint16',0};

n = n + 1;
reg(n).name = 'TIME';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'SPI',15,0,'uint16',0};
reg(n).subf{2} = {'PROCESS',31,16,'uint16',0};

n = n + 1;
reg(n).name = 'VBAT';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'VBAT',31,0,'float',0};

n = n + 1;
reg(n).name = 'VBAT_MIN';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'VBAT_MIN',31,0,'float',14.4};

n = n + 1;
reg(n).name = 'RECEIVER_BIND';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'RECEIVER_BIND',0,0,'uint8',0};

n = n + 1;
reg(n).name = 'EXPO';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'EXPO',31,0,'float',4};

n = n + 1;
reg(n).name = 'MOTOR_START';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'MOTOR_START',15,0,'uint16',50};

n = n + 1;
reg(n).name = 'MOTOR_ARMED';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'MOTOR_ARMED',15,0,'uint16',150};

n = n + 1;
reg(n).name = 'COMMAND_RATE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'COMMAND_RATE',15,0,'float',1200};

n = n + 1;
reg(n).name = 'THROTTLE_RANGE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'THROTTLE_RANGE',15,0,'float',1750};

n = n + 1;
reg(n).name = 'PITCH_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_P',31,0,'float',4};

n = n + 1;
reg(n).name = 'PITCH_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_I',31,0,'float',0.005};

n = n + 1;
reg(n).name = 'PITCH_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'ROLL_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_P',31,0,'float',4};

n = n + 1;
reg(n).name = 'ROLL_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_I',31,0,'float',0.005};

n = n + 1;
reg(n).name = 'ROLL_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'YAW_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_P',31,0,'float',6};

n = n + 1;
reg(n).name = 'YAW_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_I',31,0,'float',0.01};

n = n + 1;
reg(n).name = 'YAW_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'THROTTLE_PID_ATTEN';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'THROTTLE_PID_ATTEN',31,0,'float',0};

n = n + 1;
reg(n).name = 'RATE_PID_ATTEN';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'RATE_PID_ATTEN',31,0,'float',0};
