CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
SIZE	= arm-none-eabi-size

THUMB    =
THUMB_IW =

CFLAGS = -Wall -Wcast-align -Wcast-qual -Wimplicit
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused 
CFLAGS  += -mcpu=arm7tdmi $(THUMB_IW) -I./ -c -fno-common -g
O0 = -O0
#OS = -Os
AFLAGS  = -ahls -mapcs-32 -g -o crt.o
LDFLAGS  = -Map main.map -T linker_script-flash_memory.cmd
CPFLAGS = -O binary
ODFLAGS	= -x --syms

clean:
	-rm crt.lst crt.o main.o main.out main.map main.dmp main.bin \
	system.o timers.o isrsupport.o pwm.o ios.o motor.o \

all: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.dmp
	$(SIZE) main.out

main.out: crt.o main.o system.o timers.o isrsupport.o \
	pwm.o ios.o motor.o \
	linker_script-flash_memory.cmd
	@ echo "..linking"
	$(LD) $(LDFLAGS) -o main.out crt.o main.o system.o timers.o \
	isrsupport.o pwm.o ios.o motor.o libm.a libc.a libgcc.a

crt.o: crt.s
	$(AS) $(AFLAGS) crt.s > crt.lst

main.o: main.c
	$(CC) $(CFLAGS) $(OPT) main.c
	
system.o: system.c
	$(CC) $(CFLAGS) $(OPT) system.c
	
timers.o: timers.c
	$(CC) $(CFLAGS) $(OPT) timers.c
		
isrsupport.o: isrsupport.c
	$(CC) $(CFLAGS) $(OPT) isrsupport.c
	
pwm.o: pwm.c
	$(CC) $(CFLAGS) $(OPT) pwm.c

ios.o: ios.c
	$(CC) $(CFLAGS) $(OPT) ios.c
	
motor.o: motor.c
	$(CC) $(CFLAGS) $(OPT) motor.c							