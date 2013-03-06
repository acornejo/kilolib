all: ohc main

.PHONY: ohc main
ohc: build/ohc.elf build/ohc.hex build/ohc.lss
main: build/main.elf build/main.hex build/main.lss

AVRCC = avr-gcc
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

PFLAGS = -P usb -c avrispmkII -p m328 -U
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
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

%.o: %.c
	$(AVRCC) $(CFLAGS) -c -o $@ $<

build:
	mkdir -p $@

build/main.elf: main.c kilolib.c messages.c interrupts.h | build
	$(AVRCC) $(CFLAGS) -o $@ main.c kilolib.c messages.c

build/ohc.elf: ohc.c messages.c | build
	$(AVRCC) $(CFLAGS) $(OHC_FLAGS) -o $@ ohc.c messages.c

build/merged.hex: build/main.hex build/ohc.hex
	cat build/main.hex | grep -v ":00000001FF" > $@
	cat build/ohc.hex >> $@

program-ohc: build/merged.hex
	$(AVRUP) $(PFLAGS) "flash:w:$<:i"

program-kilo: build/main.hex
	$(AVRUP) -F $(PFLAGS) "flash:w:$<:i"

clean:
	rm -fR build
