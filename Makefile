# Minimal gcc makefile for LPC11U34

# use the arm cross compiler, not std gcc
TRGT = arm-none-eabi-
CC = $(TRGT)gcc
CXX = $(TRGT)g++
CP = $(TRGT)objcopy
DUMP = $(TRGT)objdump
CHECKSUM = ./lpc-checksum-fix/lpc-checksum-fix
TTY = /dev/ttyUSB*


ifeq ($(OS),Windows_NT)
#	CHECKSUM = ./checksum_win
	TTY = /dev/ttyS*
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Linux)
#		CHECKSUM = ./checksum
		TTY = /dev/ttyUSB*
	endif
	ifeq ($(UNAME_S),Darwin)
#		CHECKSUM = ./checksum_darwin
		TTY = /dev/tty.usb*
	endif
endif

# compiler and linker settings
COMMONFLAGS = -Wall -Wpedantic -Wextra -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wdouble-promotion -Wshadow -Wno-unused-parameter -DCORE_M0 -mcpu=cortex-m0 -I./ -Ilpc_chip_11uxx_lib -Ilpc_chip_11uxx_lib/inc -Os -ggdb
CFLAGS = $(COMMONFLAGS) -Wjump-misses-init -std=c11
CXXFLAGS = $(COMMONFLAGS) -std=c++14 -fno-rtti -fno-exceptions 
LDFLAGS = -Xlinker -print-memory-usage -Wl,--gc-sections,--script=LPC11U34_311.ld -nostartfiles

OBJS= main.o sysinit.o cr_startup_lpc11xx.o printf.o lpc_chip_11uxx_lib/src/sysinit_11xx.o lpc_chip_11uxx_lib/src/chip_11xx.o lpc_chip_11uxx_lib/src/gpio_11xx_1.o lpc_chip_11uxx_lib/src/gpio_11xx_2.o lpc_chip_11uxx_lib/src/gpiogroup_11xx.o lpc_chip_11uxx_lib/src/timer_11xx.o lpc_chip_11uxx_lib/src/pmu_11xx.o lpc_chip_11uxx_lib/src/ssp_11xx.o lpc_chip_11uxx_lib/src/clock_11xx.o lpc_chip_11uxx_lib/src/adc_11xx.o lpc_chip_11uxx_lib/src/timer_11xx.o lpc_chip_11uxx_lib/src/i2c_11xx.o lpc_chip_11uxx_lib/src/uart_11xx.o lpc_chip_11uxx_lib/src/iocon_11xx.o lpc_chip_11uxx_lib/src/pinint_11xx.o lpc_chip_11uxx_lib/src/ring_buffer.o lpc_chip_11uxx_lib/src/sysctl_11xx.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# default target
upload: firmware.bin ./lpc21isp/lpc21isp
	./lpc21isp/lpc21isp -control -donotstart -bin $< $(TTY) 115200 0

#upload: firmware.hex
#	./lpc21isp/lpc21isp -control $< $(TTY) 115200 0

leddebug: leddebug.cpp
	c++ -o $@ $<

dump: firmware.elf
	$(DUMP) -d $< > firmware.s

firmware.elf: $(OBJS)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $^

%.bin: %.elf
	$(CP) -O binary $< $@.nochksum.bin
	$(CHECKSUM) $@.nochksum.bin $@

%.hex: %.bin
	$(CP) -I binary $< -O ihex $@

clean:
	rm -f */*/*.o */*.o *.o *.elf *.bin *.s ./lpc21isp/lpc21isp 

# these target names don't represent real files
.PHONY: upload dump clean ./lpc21isp/lpc21isp

./lpc21isp/lpc21isp:
	$(MAKE) -C ./lpc21isp
