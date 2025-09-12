# Compiler and tools
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
STLINK = /home/tools/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
DFU = sudo dfu-util
LDFLAGS = *.o -lm -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs --specs=nosys.specs
#KEIL:-mthumb-interwork -nostartfiles
CFLAGS = -c -Wall -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs -Wdouble-promotion -O #--fsingle-precision-constant
#KEIL:-gdwarf-2 -MD -O -mapcs-frame -mthumb-interwork -D__GCC -D__GCC_VERSION="821"

# Drone board and feature flags
ONESHOT = 0
DSHOT = 1
PWM = 2
ifeq ($(DRONE),warpquad)
   BOARD = motof3
   FC_FLAGS = -DM1_CCW -DESC=$(ONESHOT) -DBEEPER -DVBAT
else ifeq ($(DRONE),alien6)
   BOARD = cyclone
   FC_FLAGS = -DM1_CCW -DESC=$(DSHOT) -DSHOT_RATE=600 -DBEEPER -DVBAT -DOSD -DRUNCAM -DSMART_AUDIO -DLED=4
else ifeq ($(DRONE),practice)
   BOARD = motof3
   FC_FLAGS = -DESC=$(DSHOT) -DSHOT_RATE=600 -DBEEPER -DVBAT -DIBAT -DVBAT_USE_RSSI -DOSD -DRUNCAM -DSMART_AUDIO -DLED=4
else ifeq ($(DRONE),f4)
   BOARD = f405wing
   FC_FLAGS = -DESC=$(PWM) -DSHOT_RATE=600 -DDUAL_LED_STATUS
endif

# Source list
USB_OBJ = usb.o usbd_cdc_if.o usbd_cdc.o usbd_conf.o usbd_core.o usbd_ctlreq.o usbd_desc.o usbd_ioreq.o
ifeq ($(DRONE),usb2spi)
   BOARD = revolution_usb2spi
   FC_OBJ = usb2spi.o $(BOARD).o utils.o
else
   FC_OBJ = fc.o $(BOARD).o sensor.o radio.o reg.o utils.o osd.o smart_audio.o
endif

# STM32 sources, flags and link files
ifeq ($(BOARD),nucleo)
   STM32_FLAGS = -DSTM32F3 -DSTM32F303x8 -Ic/stm32f3/cmsis/inc
   LD_SCRIPT = c/stm32f3/ldscripts/stm32f303k8tx.ld
   FC_OBJ += system_stm32f3xx.o startup_stm32f303x8.o
else ifeq ($(BOARD),revolution)
   STM32_FLAGS = -DSTM32F4 -DSTM32F405xx -Ic/stm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Ic/stm32f4/hal/inc
   LD_SCRIPT = c/stm32f4/ldscripts/stm32f405rgtx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f405xx.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else ifeq ($(BOARD),f405wing)
   STM32_FLAGS = -DSTM32F4 -DSTM32F405xx -Ic/stm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Ic/stm32f4/hal/inc
   LD_SCRIPT = c/stm32f4/ldscripts/stm32f405rgtx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f405xx.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else ifeq ($(BOARD),toothpick)
   STM32_FLAGS = -DSTM32F4 -DSTM32F411xE -Ic/stm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Ic/stm32f4/hal/inc
   LD_SCRIPT = c/stm32f4/ldscripts/stm32f411cetx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f411xe.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else ifeq ($(BOARD),revolution_usb2spi)
   STM32_FLAGS = -DSTM32F4 -DSTM32F405xx -Ic/stm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Ic/stm32f4/hal/inc
   LD_SCRIPT = c/stm32f4/ldscripts/stm32f405rgtx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f405xx.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else
   STM32_FLAGS = -DSTM32F3 -DSTM32F303xC -Ic/stm32f3/cmsis/inc -DHAL_PCD_MODULE_ENABLED -Ic/stm32f3/hal/inc
   LD_SCRIPT = c/stm32f3/ldscripts/stm32f303cctx.ld
   FC_OBJ += system_stm32f3xx.o startup_stm32f303xc.o $(USB_OBJ) stm32f3xx_hal_pcd.o stm32f3xx_hal_pcd_ex.o
endif

# Board flags
ifeq ($(BOARD),nucleo)
   FC_FLAGS += -DSENSOR=9150 -DSENSOR_ORIENTATION=0
else ifeq ($(BOARD),motof3)
   FC_FLAGS += -DSENSOR=6050 -DSENSOR_ORIENTATION=90
else ifeq ($(BOARD),cyclone)
   FC_FLAGS += -DSENSOR=6000 -DSENSOR_ORIENTATION=90
else ifeq ($(BOARD),magnum)
   FC_FLAGS += -DSENSOR=6000 -DSENSOR_ORIENTATION=0
else ifeq ($(BOARD),revolution)
   FC_FLAGS += -DSENSOR=6000 -DSENSOR_ORIENTATION=180
else ifeq ($(BOARD),toothpick)
   FC_FLAGS += -DSENSOR=6000 -DSENSOR_ORIENTATION=90
else
   FC_FLAGS += -DSENSOR=6000 -DSENSOR_ORIENTATION=0
endif

# Check current drone. If different, clean before compile
CURRENT_DRONE = $(shell cat CURRENT_DRONE)
ifneq ($(CURRENT_DRONE),$(DRONE))
all:
	echo $(DRONE) >CURRENT_DRONE
	make clean $(DRONE).elf
else
all: $(DRONE).elf
endif

flash:
	$(OBJCOPY) $(CURRENT_DRONE).elf fc.hex -O ihex
	$(STLINK) -c port=SWD reset=HWrst -w fc.hex 0x08000000 -hardRst
dfu:
	$(OBJCOPY) $(CURRENT_DRONE).elf fc.bin -O binary
	$(DFU) -a 0 -s 0x08000000$(DFU_OPT) -D fc.bin

$(DRONE).elf: $(FC_OBJ)
	$(CC) -o $(DRONE).elf -T $(LD_SCRIPT) $(LDFLAGS)
	$(SIZE) $(DRONE).elf

%.o: c/src/%.c c/inc/*.h
	$(CC) $< -Ic/inc -Ic/usb_vcp/inc $(CFLAGS) $(STM32_FLAGS) $(FC_FLAGS)

%.o: c/stm32f3/cmsis/src/%.s
	$(CC) $< $(CFLAGS)

%.o: c/stm32f4/cmsis/src/%.s
	$(CC) $< $(CFLAGS)

system_stm32f3xx.o: c/stm32f3/cmsis/src/system_stm32f3xx.c c/stm32f3/cmsis/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

system_stm32f4xx.o: c/stm32f4/cmsis/src/system_stm32f4xx.c c/stm32f4/cmsis/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: c/stm32f3/hal/src/%.c c/stm32f3/hal/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: c/stm32f4/hal/src/%.c c/stm32f4/hal/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: c/usb_vcp/src/%.c c/usb_vcp/inc/*.h
	$(CC) $< -Ic/inc -Ic/usb_vcp/inc $(CFLAGS) $(STM32_FLAGS)

clean:
	rm -rf *.o fc.hex fc.bin
