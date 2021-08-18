n = 0;
clear reg

n = n + 1;
reg(n).name = 'VERSION';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'VERSION',7,0,'uint8',0};

n = n + 1;
reg(n).name = 'STATUS';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'STATUS',7,0,'uint8',0};

n = n + 1;
reg(n).name = 'CTRL';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'SENSOR_HOST_CTRL',0,0,'uint8',0};
reg(n).subf{2} = {'ARM_TEST',2,1,'uint8',0};
reg(n).subf{3} = {'BEEP_TEST',3,3,'uint8',0};
reg(n).subf{4} = {'SENSOR_CAL',4,4,'uint8',0};
reg(n).subf{5} = {'DEBUG',5,5,'uint8',0};
reg(n).subf{6} = {'RF_HOST_CTRL',6,6,'uint8',0};
reg(n).subf{7} = {'DEBUG_RADIO',7,7,'uint8',0};
reg(n).subf{8} = {'OSD_HOST_CTRL',8,8,'uint8',0};
reg(n).subf{9} = {'SMA_HOST_CTRL',9,9,'uint8',0};

n = n + 1;
reg(n).name = 'MOTOR_TEST';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'VALUE',15,0,'uint16',0};
reg(n).subf{2} = {'SELECT',19,16,'uint8',0};
reg(n).subf{3} = {'TELEMETRY',20,20,'uint8',0};

n = n + 1;
reg(n).name = 'ERROR';
reg(n).read_only = 1;
reg(n).flash = 0;
reg(n).subf{1} = {'SENSOR',7,0,'uint8',0};
reg(n).subf{2} = {'RADIO',15,8,'uint8',0};
reg(n).subf{3} = {'RF',23,16,'uint8',0};
reg(n).subf{4} = {'CRC',31,24,'uint8',0};

n = n + 1;
reg(n).name = 'VBAT_MIN';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'VBAT_MIN',31,0,'single',3.7};

n = n + 1;
reg(n).name = 'TIME_CONSTANT';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'ACCEL',15,0,'uint16',2000};
reg(n).subf{2} = {'VBAT',31,16,'uint16',5000};

n = n + 1;
reg(n).name = 'TIME_CONSTANT_RADIO';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'TIME_CONSTANT_RADIO',15,0,'uint16',100};

n = n + 1;
reg(n).name = 'EXPO_PITCH_ROLL';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'EXPO_PITCH_ROLL',31,0,'single',3};

n = n + 1;
reg(n).name = 'EXPO_YAW';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'EXPO_YAW',31,0,'single',2};

n = n + 1;
reg(n).name = 'MOTOR';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'START',9,0,'uint16',50};
reg(n).subf{2} = {'ARMED',19,10,'uint16',200};
reg(n).subf{3} = {'RANGE',30,20,'uint16',1700};

n = n + 1;
reg(n).name = 'RATE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'PITCH_ROLL',11,0,'uint16',900};
reg(n).subf{2} = {'YAW',23,12,'uint16',540};
reg(n).subf{3} = {'ANGLE',31,24,'uint8',45};

n = n + 1;
reg(n).name = 'P_PITCH';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'P_PITCH',31,0,'single',2};

n = n + 1;
reg(n).name = 'I_PITCH';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_PITCH',31,0,'single',0.005};

n = n + 1;
reg(n).name = 'D_PITCH';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'D_PITCH',31,0,'single',0};

n = n + 1;
reg(n).name = 'P_ROLL';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'P_ROLL',31,0,'single',2};

n = n + 1;
reg(n).name = 'I_ROLL';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_ROLL',31,0,'single',0.005};

n = n + 1;
reg(n).name = 'D_ROLL';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'D_ROLL',31,0,'single',0};

n = n + 1;
reg(n).name = 'P_YAW';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'P_YAW',31,0,'single',4};

n = n + 1;
reg(n).name = 'I_YAW';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_YAW',31,0,'single',0.01};

n = n + 1;
reg(n).name = 'D_YAW';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'D_YAW',31,0,'single',0};

n = n + 1;
reg(n).name = 'P_PITCH_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'P_PITCH_ANGLE',31,0,'single',5};

n = n + 1;
reg(n).name = 'I_PITCH_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_PITCH_ANGLE',31,0,'single',0};

n = n + 1;
reg(n).name = 'D_PITCH_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'D_PITCH_ANGLE',31,0,'single',500};

n = n + 1;
reg(n).name = 'P_ROLL_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'P_ROLL_ANGLE',31,0,'single',5};

n = n + 1;
reg(n).name = 'I_ROLL_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_ROLL_ANGLE',31,0,'single',0};

n = n + 1;
reg(n).name = 'D_ROLL_ANGLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'D_ROLL_ANGLE',31,0,'single',500};

n = n + 1;
reg(n).name = 'GYRO_DC_XY';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'X',15,0,'int16',0};
reg(n).subf{2} = {'Y',31,16,'int16',0};

n = n + 1;
reg(n).name = 'GYRO_DC_Z';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'GYRO_DC_Z',15,0,'int16',0};

n = n + 1;
reg(n).name = 'ACCEL_DC_XY';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'X',15,0,'int16',0};
reg(n).subf{2} = {'Y',31,16,'int16',0};

n = n + 1;
reg(n).name = 'ACCEL_DC_Z';
reg(n).read_only = 1;
reg(n).flash = 1;
reg(n).subf{1} = {'ACCEL_DC_Z',15,0,'int16',0};

n = n + 1;
reg(n).name = 'THROTTLE';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'IDLE',15,0,'uint16',1000};
reg(n).subf{2} = {'RANGE',31,16,'uint16',1000};

n = n + 1;
reg(n).name = 'AILERON';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'IDLE',15,0,'uint16',1500};
reg(n).subf{2} = {'RANGE',31,16,'uint16',500};

n = n + 1;
reg(n).name = 'ELEVATOR';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'IDLE',15,0,'uint16',1500};
reg(n).subf{2} = {'RANGE',31,16,'uint16',500};

n = n + 1;
reg(n).name = 'RUDDER';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'IDLE',15,0,'uint16',1500};
reg(n).subf{2} = {'RANGE',31,16,'uint16',500};

n = n + 1;
reg(n).name = 'AUX';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'IDLE',15,0,'uint16',1000};
reg(n).subf{2} = {'RANGE',31,16,'uint16',1000};

n = n + 1;
reg(n).name = 'MPU_CFG';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'FILT',7,0,'uint8',1};
reg(n).subf{2} = {'RATE',15,8,'uint8',0};

n = n + 1;
reg(n).name = 'FC_CFG';
reg(n).read_only = 0;
reg(n).flash = 1;
reg(n).subf{1} = {'I_TRANSFER',0,0,'uint8',1};

n = n + 1;
reg(n).name = 'DEBUG_INT';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'DEBUG_INT',31,0,'uint32',0};

n = n + 1;
reg(n).name = 'DEBUG_FLOAT';
reg(n).read_only = 0;
reg(n).flash = 0;
reg(n).subf{1} = {'DEBUG_FLOAT',31,0,'single',0};
