CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
SIZE	= arm-none-eabi-size

THUMB    =
THUMB_IW =

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
OPT = s

CFLAGS = -Wall -Wcast-align -Wcast-qual -Wimplicit
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused 
CFLAGS += -mcpu=arm7tdmi $(THUMB_IW) -I./ -c -fno-common -g
CFLAGS += -O$(OPT)

AFLAGS  = -ahls -mapcs-32 -g -o crt.o
LDFLAGS  = -Map main.map -T linker_script-flash_memory.cmd
CPFLAGS = -O binary
ODFLAGS	= -x --syms

clean:
	-rm crt.lst crt.o main.o main.out main.map main.dmp main.bin \
	system.o timers.o isrsupport.o pwm.o ios.o bldc.o adc.o\

all: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.dmp
	$(SIZE) main.out

main.out: crt.o main.o system.o timers.o isrsupport.o \
	pwm.o ios.o bldc.o \
	linker_script-flash_memory.cmd
	@ echo "..linking"
	$(LD) $(LDFLAGS) -o main.out crt.o main.o system.o timers.o \
	isrsupport.o pwm.o ios.o bldc.o adc.o libc.a libgcc.a libm.a

crt.o: crt.s
	$(AS) $(AFLAGS) crt.s > crt.lst

main.o: main.c
	$(CC) $(CFLAGS) main.c
	
system.o: system.c
	$(CC) $(CFLAGS) system.c
	
timers.o: timers.c
	$(CC) $(CFLAGS) timers.c
		
isrsupport.o: isrsupport.c
	$(CC) $(CFLAGS) isrsupport.c
	
pwm.o: pwm.c
	$(CC) $(CFLAGS) pwm.c

ios.o: ios.c
	$(CC) $(CFLAGS) ios.c
	
bldc.o: bldc.c
	$(CC) $(CFLAGS) bldc.c
	
adc.o: adc.c
	$(CC) $(CFLAGS) adc.c	