all: ohc program bootldr 

.PHONY: ohc program bootldr lib
lib: build/kilolib.a
ohc: build/ohc.elf build/ohc.hex build/ohc.lss
program: build/program.elf build/program.hex build/program.lss
bootldr: build/bootldr.elf build/bootldr.hex build/bootldr.lss

CC = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

PFLAGS = -P usb -c avrispmkII -U
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -DF_CPU=8000000
ASFLAGS = $(CFLAGS)
BOOTLDR_FLAGS = -Wl,-section-start=.text=0x7000 -DBOOTLOADER
OHC_FLAGS = -Wl,-section-start=.text=0x7000 -DOHC

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0  

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

%.eep: %.elf
	$(AVROC) -O ihex $(EEPROM) $< $@

%.bin: %.elf
	$(AVROC) -O binary $(FLASH) $< $@ 

build:
	mkdir -p $@

build/kilolib.a: kilolib.o message_crc.o message_send.o | build
	$(AVRAR) rcs $@ kilolib.o message_crc.o message_send.o
	rm -f *.o

build/program.elf: program.c kilolib.c message_crc.c message_send.S | build
	$(CC) $(CFLAGS) -o $@ program.c kilolib.c message_crc.c message_send.S

build/ohc.elf: ohc.c message_crc.c message_send.S | build
	$(CC) $(CFLAGS) $(OHC_FLAGS) -o $@ ohc.c message_crc.c message_send.S

build/bootldr.elf: bootldr.c kilolib.c message_crc.c | build
	$(CC) $(CFLAGS) $(BOOTLDR_FLAGS) -o $@ bootldr.c kilolib.c message_crc.c

build/kilo-merged.hex: build/program.hex build/bootldr.hex
	cat build/program.hex | grep -v ":00000001FF" > $@
	cat build/bootldr.hex >> $@

program-ohc: build/ohc.hex
	$(AVRUP) -p m328  $(PFLAGS) "flash:w:$<:i"

program-boot: build/bootldr.hex
	$(AVRUP) -p m328p $(PFLAGS) "flash:w:$<:i"

program-kilo: build/kilo-merged.hex
	$(AVRUP) -p m328p $(PFLAGS) "flash:w:$<:i"

clean:
	rm -fR build
