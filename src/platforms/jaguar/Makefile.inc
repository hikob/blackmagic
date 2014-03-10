CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

DEVICE = EFM32LG940F256

# This is already set
#CFLAGS += -Wall -Wextra -Wno-pointer-sign -Wno-char-subscripts\
#    -Wno-sign-compare \
#    -O2 -std=gnu99 -g3 -DBUILDDATE=\"$(BUILDDATE)\"\
#    -I. -Iinclude -I$(PLATFORM_DIR) \
#    -DVERSION_SUFFIX=\"`../scripts/setlocalversion`\" -MD


CFLAGS += -DENERGYMICRO -D$(DEVICE) \
    -mcpu=cortex-m3 -mthumb \
    -mfix-cortex-m3-ldrd -ffunction-sections \
    -fdata-sections -fomit-frame-pointer \
    -fwide-exec-charset=UTF-16LE -fshort-wchar \
    -DDEBUG_EFM_USER
 
ASMFLAGS += -x assembler-with-cpp -D$(DEVICE) -Wall -Wextra -mcpu=cortex-m3 -mthumb

LDFLAGS += -mcpu=cortex-m3 -mthumb \
    -T../energymicro/Device/EnergyMicro/EFM32LG/Source/GCC/efm32lg.ld \
    -Wl,--gc-sections,--no-wchar-size-warning \
    -Wl,--start-group -lgcc -lc -lnosys   -Wl,--end-group -Wl,-Ttext=0x00004000

INCLUDEPATHS += \
-I../energymicro/CMSIS/Include \
-I../energymicro/Device/EnergyMicro/EFM32LG/Include \
-I../energymicro/emlib/inc \
-I../energymicro/usb/inc

CFLAGS += -Iplatforms/efm32  $(INCLUDEPATHS)

VPATH += ../energymicro/Device/EnergyMicro/EFM32LG/Source/GCC/ \
    ../energymicro/Device/EnergyMicro/EFM32LG/Source/ \
    ../energymicro/emlib/src/ \
    ../energymicro/usb/src/

VPATH += platforms/efm32

SRC += \
    jaguar.c \
    system_efm32lg.c \
    em_assert.c \
    em_cmu.c \
    em_dma.c \
    em_ebi.c \
    em_emu.c \
    em_gpio.c \
    em_i2c.c \
    em_int.c \
    em_lcd.c \
    em_system.c \
    em_timer.c \
    em_usart.c \
    em_usbd.c \
    em_usbdch9.c \
    em_usbhal.c \
    em_usbdep.c \
    em_usbdint.c \
    em_usbtimer.c \
    \
    dmactrl.c \
    usbuart.c \
    usbpowercon.c
    

S_SRC +=  startup_efm32lg.S

all:	blackmagic.bin 

blackmagic.bin:	blackmagic
	$(OBJCOPY) -O binary $^ $@

host_clean:
	-rm -f blackmagic.bin
