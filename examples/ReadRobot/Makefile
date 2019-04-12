

# Select the serial port on Windows
PORT=

# Select the robot (MINITAUR, or MINITAUR_E)
ROBOT=MINITAUR_E


# -- Don't modify below here --

# Autodetect port on Mac and Linux
ifeq ($(shell uname),Darwin)
	PORT=$(shell ls /dev/tty.usbserial* 2> /dev/null)
else ifeq ($(shell uname),Linux)
	PORT=$(shell ls /dev/ttyUSB*  2> /dev/null)
	SUDO=sudo 
endif
ifeq ($(PORT),)
    $(error No robot found! Edit Makefile to set the port)
endif

# Use correct firmware
ifeq ($(ROBOT),MINITAUR)
	LINKER_SCRIPT=STM32F303VC_FLASH.ld
	SDKLIB=-lmblc
else
	LINKER_SCRIPT=STM32F446ZETx_FLASH.ld
	SDKLIB=-lmbm
endif

# Flags
SDK_LOCATION=../..
BFLAGS=-mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra -g -DROBOT_$(ROBOT)
CFLAGS=-DNDEBUG -DARM_MATH_CM4 -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -I$(SDK_LOCATION)/inc -I$(SDK_LOCATION)/thirdparty/nanopb 
LDFLAGS=-T $(SDK_LOCATION)/lib/$(LINKER_SCRIPT) -nostartfiles -Xlinker --gc-sections -L$(SDK_LOCATION)/lib --specs=nano.specs -u _printf_float -Wl,-whole-archive $(SDKLIB) -Wl,-no-whole-archive
LDLIBS=-larm_cortexM4lf_math
ARCH=arm-none-eabi-
UPLOAD=python $(SDK_LOCATION)/tools/stm32loader.py -p $(PORT) -b 230400 -y mblc -E 524288 -L 0 -ew main.bin && echo Done.

all:
	@$(ARCH)g++ $(BFLAGS) $(CFLAGS) -c -o main.o main.cpp
	@$(ARCH)g++ $(BFLAGS) $(LDFLAGS)  -o main.elf main.o $(LDLIBS)
	@$(ARCH)size --format=berkeley main.elf
	@$(ARCH)objcopy --output-target=binary main.elf main.bin
	@$(SUDO) $(UPLOAD)

clean:
	rm -f main.elf main.bin main.o
