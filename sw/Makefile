CC = /mnt/c/wsl/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc
OBJCOPY = /mnt/c/wsl/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-objcopy
SIZE = /mnt/c/wsl/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-size
STLINK = "/mnt/c/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe"
DFU = /mnt/c/programs/dfu-util-0.9/dfu-util.exe
LDFLAGS = *.o -lm -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs --specs=nosys.specs
#-L/home/mleurent/gcc-arm-none-eabi-8-2018-q4-major/arm-none-eabi/lib -L/home/mleurent/gcc-arm-none-eabi-8-2018-q4-major/lib/gcc/arm-none-eabi/8.2.1
#KEIL:-mthumb-interwork -nostartfiles
CFLAGS = -c -Wall -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs -Wdouble-promotion -O
#--fsingle-precision-constant
#KEIL:-I/home/mleurent/gcc-arm-none-eabi-8-2018-q4-major/arm-none-eabi/include -I/home/mleurent/gcc-arm-none-eabi-8-2018-q4-major/lib/gcc/arm-none-eabi/8.2.1/include
#KEIL:-gdwarf-2 -MD -O -mapcs-frame -mthumb-interwork -D__GCC -D__GCC_VERSION="821"

USB_OBJ = usb.o usbd_cdc_if.o usbd_cdc.o usbd_conf.o usbd_core.o usbd_ctlreq.o usbd_desc.o usbd_ioreq.o
FC_OBJ = fc.o $(BOARD).o sensor.o radio.o reg.o utils.o
ifeq ($(BOARD),nucleo)
   STM32_FLAGS = -DSTM32F3 -DSTM32F303x8 -Istm32f3/cmsis/inc
   LD_SCRIPT = stm32f3/ldscripts/stm32f303k8tx.ld
   FC_OBJ += system_stm32f3xx.o startup_stm32f303x8.o
else ifeq ($(BOARD),revolution)
   STM32_FLAGS = -DSTM32F4 -DSTM32F405xx -Istm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Istm32f4/hal/inc
   LD_SCRIPT = stm32f4/ldscripts/stm32f405rgtx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f405xx.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else ifeq ($(BOARD),toothpick)
   STM32_FLAGS = -DSTM32F4 -DSTM32F411xE -Istm32f4/cmsis/inc -DHAL_PCD_MODULE_ENABLED -DHAL_RCC_MODULE_ENABLED -Istm32f4/hal/inc
   LD_SCRIPT = stm32f4/ldscripts/stm32f411cetx.ld
   FC_OBJ += system_stm32f4xx.o startup_stm32f411xe.o $(USB_OBJ) stm32f4xx_hal_pcd.o stm32f4xx_hal_pcd_ex.o stm32f4xx_ll_usb.o
else
   STM32_FLAGS = -DSTM32F3 -DSTM32F303xC -Istm32f3/cmsis/inc -DHAL_PCD_MODULE_ENABLED -Istm32f3/hal/inc
   LD_SCRIPT = stm32f3/ldscripts/stm32f303cctx.ld
   FC_OBJ += system_stm32f3xx.o startup_stm32f303xc.o $(USB_OBJ) stm32f3xx_hal_pcd.o stm32f3xx_hal_pcd_ex.o
endif

ONESHOT = 0
DSHOT   = 1
ESC ?= $(DSHOT)
ifeq ($(BOARD),nucleo)
   FC_FLAGS = -DSENSOR=9150 -DESC=$(ESC) -DSENSOR_ORIENTATION=0
else ifeq ($(BOARD),motof3)
   FC_FLAGS = -DSENSOR=6050 -DESC=$(ESC) -DSENSOR_ORIENTATION=90 -DBEEPER -DVBAT
else ifeq ($(BOARD),cyclone)
   FC_FLAGS = -DSENSOR=6000 -DESC=$(ESC) -DSENSOR_ORIENTATION=90 -DBEEPER -DVBAT
else ifeq ($(BOARD),magnum)
   FC_FLAGS = -DSENSOR=6000 -DESC=$(ESC) -DSENSOR_ORIENTATION=0 -DBEEPER -DVBAT
else ifeq ($(BOARD),revolution)
   FC_FLAGS = -DSENSOR=6000 -DESC=$(ESC) -DSENSOR_ORIENTATION=180 #-DRF
else ifeq ($(BOARD),toothpick)
   FC_FLAGS = -DSENSOR=6000 -DESC=$(ESC) -DSENSOR_ORIENTATION=90 -DVBAT -DRUNCAM
else
   FC_FLAGS = -DSENSOR=6000 -DESC=$(ESC) -DSENSOR_ORIENTATION=0
endif

CURRENT_BOARD = $(shell cat CURRENT_BOARD)

ifneq ($(CURRENT_BOARD),$(BOARD))
all:
	echo $(BOARD) >CURRENT_BOARD
	make clean $(BOARD).elf
else
all: $(BOARD).elf
endif

flash:
	$(OBJCOPY) $(CURRENT_BOARD).elf fc.hex -O ihex
	$(STLINK) -P fc.hex -Rst

dfu:
	$(OBJCOPY) $(CURRENT_BOARD).elf fc.bin -O binary
	$(DFU) -a 0 -s 0x08000000$(DFU_OPT) -D fc.bin

$(BOARD).elf: $(FC_OBJ)
	$(CC) -o $(BOARD).elf -T $(LD_SCRIPT) $(LDFLAGS)
	$(SIZE) $(BOARD).elf

%.o: src/%.c inc/*.h
	$(CC) $< -Iinc -Iusb_vcp/inc $(CFLAGS) $(STM32_FLAGS) $(FC_FLAGS)

%.o: stm32f3/cmsis/src/%.s
	$(CC) $< $(CFLAGS)

%.o: stm32f4/cmsis/src/%.s
	$(CC) $< $(CFLAGS)

system_stm32f3xx.o: stm32f3/cmsis/src/system_stm32f3xx.c stm32f3/cmsis/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

system_stm32f4xx.o: stm32f4/cmsis/src/system_stm32f4xx.c stm32f4/cmsis/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: stm32f3/hal/src/%.c stm32f3/hal/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: stm32f4/hal/src/%.c stm32f4/hal/inc/*.h
	$(CC) $< $(CFLAGS) $(STM32_FLAGS)

%.o: usb_vcp/src/%.c usb_vcp/inc/*.h
	$(CC) $< -Iinc -Iusb_vcp/inc $(CFLAGS) $(STM32_FLAGS)

clean:
	rm -rf *.o fc.hex fc.bin
