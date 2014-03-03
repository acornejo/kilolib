all: ohc bootldr blank 

.PHONY: ohc bootldr blank
KILOLIB = build/kilolib.a
ohc: build/ohc.elf build/ohc.hex build/ohc.lss
bootldr: build/bootldr.elf build/bootldr.hex build/bootldr.lss
blank: build/blank.elf build/blank.hex build/blank.lss

CC = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

# PFLAGS = -P /dev/ttyACM0 -c avrisp2 -U
PFLAGS = -P usb -c avrispmkII -U
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -DF_CPU=8000000
ASFLAGS = $(CFLAGS)
BOOTLDR_FLAGS = -Wl,-section-start=.text=0x7000 -DBOOTLOADER
OHC_FLAGS = -Wl,-section-start=.text=0x7000 -DOHC

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0  

define MERGE 
	@cat $(1) | grep -v ":00000001FF" > $(3)
	@cat $(2) >> $(3)
endef

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

$(KILOLIB): kilolib.o message_crc.o message_send.o | build
	$(AVRAR) rcs $@ kilolib.o message_crc.o message_send.o 
	rm -f *.o

build/blank.elf: blank.c $(KILOLIB) | build
	$(CC) $(CFLAGS) -o $@ $< $(KILOLIB) 

build/ohc.elf: ohc.c message_crc.c message_send.S | build
	$(CC) $(CFLAGS) $(OHC_FLAGS) -o $@ ohc.c message_crc.c message_send.S

build/bootldr.elf: bootldr.c kilolib.c message_crc.c | build
	$(CC) $(CFLAGS) $(BOOTLDR_FLAGS) -o $@ bootldr.c kilolib.c message_crc.c

program-ohc: build/ohc.hex
	$(AVRUP) -p m328  $(PFLAGS) "flash:w:$<:i"

program-boot: build/bootldr.hex
	$(AVRUP) -p m328p $(PFLAGS) "flash:w:$<:i"

program-blank: build/blank.hex build/bootldr.hex
	$(call MERGE, "build/blank.hex", "build/bootldr.hex", "build/merged-blank.hex")
	$(AVRUP) -p m328p $(PFLAGS) "flash:w:build/merged-blank.hex:i"

program-demo: build/demo.hex build/bootldr.hex
	$(call MERGE, "build/demo.hex", "build/bootldr.hex", "build/merged-demo.hex")
	$(AVRUP) -p m328p $(PFLAGS) "flash:w:build/merged-demo.hex:i"

clean:
	rm -fR build
