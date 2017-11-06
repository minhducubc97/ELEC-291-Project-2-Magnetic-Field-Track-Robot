PORTN=$(shell type COMPORT.inc)

usart.elf: usart.o
	avr-gcc -mmcu=atmega328p usart.o -o usart.elf
	avr-objcopy -j .text -j .data -O ihex usart.elf usart.hex
	@echo done!
		
usart.o: usart.c usart.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c usart.c

clean:
	@del *.hex *.elf *.o 2>nul

LoadFlash:
	@Taskkill /IM putty.exe /F 2>nul | wait 500
	spi_atmega328 -p -v -crystal usart.hex

putty:
	@Taskkill /IM putty.exe /F 2>nul | wait 500
	c:\putty\putty.exe -serial $(PORTN) -sercfg 19200,8,n,1,N -v

dummy: usart.hex
	@echo Hello dummy!
	
explorer:
	explorer .
