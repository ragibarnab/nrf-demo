## Your project
TARGET = transmitter
LIBDIR = ../lib
SOURCES = $(wildcard *.c $(LIBDIR)/*.c)
OBJECTS = $(SOURCES:.c=.o)


## Chip and project-specific global definitions
MCU   = atmega328p
F_CPU = 1000000UL 
BAUD  = 9600UL
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

## Cross-compilation
CC = avr-gcc
OBJCOPY = avr-objcopy

## Compiler/linker options
CFLAGS = -Os -g -std=gnu99 -Wall
# CFLAGS += -funsigned-char -funsigned-bitfields 
# CFLAGS += -fpack-struct -fshort-enums 
CFLAGS += -ffunction-sections -fdata-sections 

TARGET_ARCH = -mmcu=$(MCU)

LDFLAGS = -Wl,-Map,$(TARGET).map 
LDFLAGS += -Wl,--gc-sections 

## Targets and rules
all: flash

%.hex: %.elf
	 $(OBJCOPY) -j .text -j .data -O ihex $< $@

flash: $(TARGET).hex
	avrdude -c usbasp -p $(MCU) -U flash:w:$(TARGET).hex

%.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean: 
## for Windows, rm was replaced with del
	del -f $(TARGET).elf $(TARGET).hex $(TARGET).map  

