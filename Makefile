all: ohc program bootldr

.PHONY: ohc program bootldr
ohc: build/ohc.elf build/ohc.hex build/ohc.lss
program: build/program.elf build/program.hex build/program.lss
bootldr: build/bootldr.elf build/bootldr.hex build/bootldr.lss

AVRCC = avr-gcc
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

PFLAGS = -P usb -c avrispmkII -p m328 -U
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -DF_CPU=8000000
BOOTLDR_FLAGS = -Wl,-section-start=.text=0x7000 -DBOOTLOADER -DDEBUG -Wl,-u,vfprintf -lprintf_min
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

build/program.elf: program.c kilolib.c messages.c interrupts.h | build
	$(AVRCC) $(CFLAGS) -o $@ program.c kilolib.c messages.c

build/ohc.elf: ohc.c messages.c | build
	$(AVRCC) $(CFLAGS) $(OHC_FLAGS) -o $@ ohc.c messages.c

build/bootldr.elf: bootldr.c kilolib.c messages.c interrupts.h | build
	$(AVRCC) $(CFLAGS) $(BOOTLDR_FLAGS) -o $@ bootldr.c kilolib.c messages.c

build/ohc-merged.hex: build/program.hex build/ohc.hex
	cat build/program.hex | grep -v ":00000001FF" > $@
	cat build/ohc.hex >> $@

build/kilo-merged.hex: build/program.hex build/bootldr.hex
	cat build/program.hex | grep -v ":00000001FF" > $@
	cat build/bootldr.hex >> $@

program-ohc: build/ohc-merged.hex
	$(AVRUP) $(PFLAGS) "flash:w:$<:i"

program-kilo: build/kilo-merged.hex
	$(AVRUP) -F $(PFLAGS) "flash:w:$<:i"

clean:
	rm -fR build
