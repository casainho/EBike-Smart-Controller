################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../main.o \
../stm32f10x_gpio.o \
../stm32f10x_rcc.o \
../stm32f10x_usart.o \
../usart.o 

C_SRCS += \
../main.c \
../stm32f10x_gpio.c \
../stm32f10x_it.c \
../stm32f10x_lib.c \
../stm32f10x_nvic.c \
../stm32f10x_rcc.c \
../stm32f10x_usart.c \
../stm32f10x_vector.c \
../usart.c 

OBJS += \
./main.o \
./stm32f10x_gpio.o \
./stm32f10x_it.o \
./stm32f10x_lib.o \
./stm32f10x_nvic.o \
./stm32f10x_rcc.o \
./stm32f10x_usart.o \
./stm32f10x_vector.o \
./usart.o 

C_DEPS += \
./main.d \
./stm32f10x_gpio.d \
./stm32f10x_it.d \
./stm32f10x_lib.d \
./stm32f10x_nvic.d \
./stm32f10x_rcc.d \
./stm32f10x_usart.d \
./stm32f10x_vector.d \
./usart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


