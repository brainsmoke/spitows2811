
TARGET = spitows.hex
ELFTARGET = spitows.elf

.PHONY: clean

all: $(TARGET) $(ELFTARGET)


%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.elf: %.o
	avr-ld -m avr25 -o $@ $^

%.o: %.S
	avr-gcc -mmcu=attiny85 -c -o $@ $< 

flash:
	avrdude -v -c usbtiny -pt85 -U flash:w:$(TARGET)

fuses:
	avrdude -v -c usbtiny -pt85 -U lfuse:w:0xe1:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

clean:
	-rm spitows.hex spitows.elf spitows.o
