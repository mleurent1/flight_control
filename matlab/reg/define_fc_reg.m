n = 0;
clear reg

n = n + 1;
reg(n).name = 'VERSION';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'VERSION',7,0,'uint8',14};

n = n + 1;
reg(n).name = 'CTRL';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'MPU_HOST_CTRL',0,0,'uint8',0};
reg(n).subf{2} = {'RESET_INTEGRAL_ON_ARMED',1,1,'uint8',1};
reg(n).subf{3} = {'BEEP_TEST',2,2,'uint8',0};
reg(n).subf{4} = {'LED_SELECT',6,3,'uint8',2};
reg(n).subf{5} = {'MOTOR_SELECT',9,7,'uint8',0};
reg(n).subf{6} = {'MOTOR_TEST',25,10,'uint16',1000};

n = n + 1;
reg(n).name = 'DEBUG';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'DEBUG',7,0,'uint8',0};

n = n + 1;
reg(n).name = 'ERROR';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'SENSOR',15,0,'uint16',0};
reg(n).subf{2} = {'COMMMAND',31,16,'uint16',0};

n = n + 1;
reg(n).name = 'LOOP_TIME';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'LOOP_TIME',15,0,'uint16',0};

n = n + 1;
reg(n).name = 'VBAT';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'VBAT',31,0,'float',0};

n = n + 1;
reg(n).name = 'VBAT_MIN';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'VBAT_MIN',31,0,'float',10.7};

n = n + 1;
reg(n).name = 'VBAT_MAX';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'VBAT_MAX',31,0,'float',12.6};

n = n + 1;
reg(n).name = 'EXPO';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'EXPO',31,0,'float',0.2};

n = n + 1;
reg(n).name = 'MOTOR_START';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'MOTOR_START',15,0,'uint16',1050};

n = n + 1;
reg(n).name = 'MOTOR_ARMED';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'MOTOR_ARMED',15,0,'uint16',1075};

n = n + 1;
reg(n).name = 'COMMAND_RATE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'COMMAND_RATE',31,0,'float',2.5};

n = n + 1;
reg(n).name = 'THROTTLE_RANGE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'THROTTLE_RANGE',15,0,'uint16',900};

n = n + 1;
reg(n).name = 'PITCH_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_P',31,0,'float',2};

n = n + 1;
reg(n).name = 'PITCH_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_I',31,0,'float',0.01};

n = n + 1;
reg(n).name = 'PITCH_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'ROLL_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_P',31,0,'float',2};

n = n + 1;
reg(n).name = 'ROLL_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_I',31,0,'float',0.01};

n = n + 1;
reg(n).name = 'ROLL_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ROLL_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'YAW_P';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_P',31,0,'float',3};

n = n + 1;
reg(n).name = 'YAW_I';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_I',31,0,'float',0.02};

n = n + 1;
reg(n).name = 'YAW_D';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'YAW_D',31,0,'float',0};

n = n + 1;
reg(n).name = 'TPA';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'TPA',31,0,'float',0.4};
