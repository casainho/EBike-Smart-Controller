NAME   = smart_ebike_controller

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
  
CFLAGS  =  -I./ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -Tlinker_script.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

all: test

clean:
	-rm main.list main.o main.out main.bin stm32f10x_rcc.o stm32f10x_gpio.o

test: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: main.o stm32f10x_rcc.o stm32f10x_gpio.o linker_script.ld 
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.out  main.o stm32f10x_rcc.o stm32f10x_gpio.o 

stm32f10x_rcc.o: stm32f10x_rcc.c 
	@ echo ".compiling"
	 $(CC) $(CFLAGS) stm32f10x_rcc.c 
	 
stm32f10x_gpio.o: stm32f10x_gpio.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) stm32f10x_gpio.c 

main.o: main.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) main.c
