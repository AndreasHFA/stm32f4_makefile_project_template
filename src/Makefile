#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
#	this is originally from https://github.com/halherta/iotogglem0_wspl
#
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# make flash_stlink = build project files and flash the microcontroller via st-flash (https://github.com/texane/stlink.git)
#
# To rebuild project do "make clean" and "make all".
#
# This makefile offers the possibility to build STM32F4 projects without any IDE.
# Since the latest release of the GNU ARM Plugin for the eclipse IDE is a crap it gives back the
# possibilty to define own project directory structures.
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified by Franz Flasch
# Use at your own risk!!!!!
#
##############################################################################################
# Start of default section
#

# cross compiler path settings
CC_PATH		= /home/andreas/Documents/STM32/tools/gcc-arm-none-eabi-4_9-2014q4/bin/
CCPREFIX	= $(CC_PATH)arm-none-eabi-
CC   		= $(CCPREFIX)gcc
CP   		= $(CCPREFIX)objcopy
AS   		= $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI		= $(CCPREFIX)gdbtui
SZ		= $(CCPREFIX)size
HEX  		= $(CP) -O ihex
BIN  		= $(CP) -O binary -S
MCU  		= cortex-m4

# st - tools path
ST_TOOL	=/home/andreas/Documents/STM32/tools/stlink

# List all C defines here
DDEFS =

# Define optimisation level here
OPT = -Os

# Define project name and Ram/Flash mode here
PROJECT		= stm32f4_template

# Define Output
OUTPUT_DIR = output

TARGET  = 	$(OUTPUT_DIR)/$(PROJECT)

# Source directories
LINKERSCRIPT	= ./linkerscript/stm32_flash.ld

APP_INC_DIR	= 	. \
			./2_device \
			./3_application \
			./4_config

# APPLICATION SPECIFIC
SRC = ./main.c
# Further sources have to be specified with "+="
#SRC += 
SRC += ./2_device/stm32_configuration.c
SRC += ./2_device/stm32_serial_functions.c

ASM_SRC = 
# Further sources have to be specified with "+="
#ASM_SRC += 

INCDIRS = $(APP_INC_DIR)

# include submakefiles here
#include 1_peripheral/submakefile_std_lib.mk 	# STM32 Standard Peripheral Library
include submakefile_std_lib.mk 			# STM32 Standard Peripheral Library


INCDIR  = $(patsubst %,-I%, $(INCDIRS))

## run from Flash
DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1

OBJS  	= $(ASM_SRC:.s=.o) $(SRC:.c=.o)

MCFLAGS = -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst) 
CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)

# "-Xlinker --gc-sections" - removes unused code from the output binary - saves memory
LDFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb -nostartfiles -Xlinker --gc-sections -T$(LINKERSCRIPT) -Wl,-Map=$(TARGET).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)


#
# makefile rules
# 
#all: $(OBJS) $(TARGET).elf  $(TARGET).hex $(TARGET).bin
#	$(TRGT)size $(TARGET).elf

all: $(OBJS) $(TARGET).elf  $(TARGET).hex $(TARGET).bin
	$(SZ) $(TARGET).elf
 
%o: %c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%o: %s
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%hex: %elf
	$(HEX) $< $@
	
%bin: %elf
	$(BIN)  $< $@

	
flash_openocd: $(TARGET).bin
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase $(TARGET).bin 0x08000000" -c "sleep 100" -c "verify_image $(TARGET).bin 0x08000000" -c "sleep 100" -c "reset run" -c shutdown


flash_stlink: $(TARGET).bin
	$(ST_TOOL)/st-flash write $(TARGET).bin 0x8000000


erase_openocd:
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown 


erase_stlink:
	$(ST_TOOL)/st-flash erase


debug_openocd: $(TARGET).elf flash_openocd
	xterm -e openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDBTUI) --eval-command="target remote localhost:3333" $(TARGET).elf 


debug_stlink: $(TARGET).elf
	xterm -e st-util &
	$(GDBTUI) --eval-command="target remote localhost:4242"  $(TARGET).elf -ex 'load'

		
clean:
	-rm -rf $(OBJS)
	-rm -rf $(TARGET).elf
	-rm -rf $(TARGET).map
	-rm -rf $(TARGET).hex
	-rm -rf $(TARGET).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)
# *** EOF ***
