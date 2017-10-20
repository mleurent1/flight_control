# Flight Control

The first flight control software that is configured with Matlab!

I developed a Chrome app some time ago but I will not maintain it anymore (too time consuming and not as flexible as Maltab)

## Code structure

The main program is in *fc.c*. It is calling:
- generic functions (like *sensor_read* or *set_motors*) whose content is board specific and is defined in *[\board_name].c*.
Those functions are declared in *board.h* which also includes *[\board_name].h*.
- functions that post-process gyro/accel data and initialise the sensor chip: *sensor.c*
- functions that post-process the radio receiver channels: *radio.c*

In *[\board_name].h*, you can set
- the radio type:
	- IBUS: Turnigy
	- SUMD: Graupner
	- SBUS: Futaba
- the ESC protocol:
	- ONESHOT: Oneshot125
	- DSHOT: Dshot600

*[\board_name].h* also specify the CMSIS to use as well as the sensor chip/orientation

Board names are:
- revolution: Open pilot CC3D Revolution
- cyclone: Motolab Cyclone
- motof3: Motolab MotoF3
- nucleo: STM32 Nucleo 32

There are 3 sets of registers:
- The active configuration, a array in the RAM that must be initialised
- A default *const* table
- A custom table stored in the flash memory (located outside of the program)

At initialisation, if the version register from the custom table is mathing the default one, the active registers will take the custom values, otherwise they will take the default values.
The user can write to the custom table with Matlab.

## Orientation convention

### Gyro and accel vectors
```
     ^+ax
    _|_> +gy = +roll
   / |
   \_|_/
 ____|____
|   __>+gz|  __> +gx = +pitch
|  / .+az |-/----> +ay
|  \__/   | \__/
|_________|
```
pitch forward => 
- gyro x > 0
- accel x > 0
- pitch > 0
- angle pitch > 0

roll right =>
- gyro y > 0
- accel y > 0
- roll > 0
- angle roll > 0

rotation clockwise => 
- gyro z > 0
- yaw > 0

### Motors
```
     ^+ax
     |
     |
 __>   <__
/4       1\
\__/   \__/
    \  /
     \/  ----> +ay
     /\
 __ /  \__
/3 \   / 2\
\__     __/
   >   <
```
- Motor 1: front right, anti-clockwise
- Motor 2: rear right, clockwise
- Motor 3: rear left, anti-clockwise
- Motor 4: front left, clockwise

## Matlab

- *init.m*: you need to specify the COM port to open
- *deinit.m*: to close COM port
- *read_config.m*: Print the active configuration (the one in the RAM).
*fc* is the register access object, if you set fc.target = 1, you will the register values from the flash.
This is what the function *config_mismatch.m* does, by comparing the active config and the flash config
- *save_config.m*
- *debug.m*: debug(case,nb_points). Plots usefull real time data. It will stop after nb_points have been captured. A single time window is 256 points

## Calibration

- fc.CTRL__SENSOR_CAL(1): Start the DC offset calibration of the gyros and accel. Led will blink during 1s.
- fc.CTRL__RADIO_CAL_IDLE(1): Start the DC offset calibration of the radio commands. Led will blink during 1s.
- fc.CTRL__RADIO_CAL_RANGE(1): Start the full range calibration of the radio commands. Led will blink during 10s, during which you have the move the sticks in circle, to their maximum position.

The calibrated values are in the active config, use *read_config* to print them. Do not forget to *save_config*, otherwise they will be discarded next time you power up the board.
NB: Throttle: idle ~= 1000, range ~= 1000. Pitch, roll and yaw: idle ~= 1500, range ~= 500

## Check list

To do before a **first flight** or after **firmware upgrade**

Most of the tests can be done on USB 5V supply.

But if you want to test the motors or if you need the Lipo supply, **REMOVE THE PROPS !!!**

1. Check the sensor behaviour, as described in *Orientation convention*
```
debug(2,nb_points)
```
- 1rst plot is gyro
- 2nd plot is accel
- blue = x, red = y, green = z
	
2. Check the angle behaviour, as described in *Orientation convention*. Check also that angles from accel are coherent with the gyro ones.
```
debug(3,nb_points)
```
- 1rst plot is combined angles from gyro and accel
- 2nd plot is instantaneous angles from accel
- blue = pitch, red = roll

3. Check the radio commands. When you move the sicks up or right, the value should be positive.
```
debug(5,nb_points)
```
- 1rst plot: blue = pitch, red = roll, green = yaw
- 2nd plot: blue = throttle, red = arm switch
- NB: arm = 0: Motors OFF. arm = 0.5: acro mode. arm = 1: angle mode.

4. Check the motors, by either moving the quad or moving the sticks.
```
debug(7,nb_points)
```
- Do not forget to arm
- quad pitch forward: motor 1&4 up, motor 2&3 down
- quad roll right: motor 1&2 up, motor 3&4 down
- radio pitch forward: motor 2&3 up, motor 1&4 down
- radio roll right: motor 3&4 up, motor 1&2 down
