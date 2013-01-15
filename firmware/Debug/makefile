################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
EBike Smart Controller.hex \

SECONDARY_LIST += \
EBike Smart Controller.lst \

SECONDARY_SIZE += \
EBike Smart Controller.siz \


# All Target
all: EBike\ Smart\ Controller.elf secondary-outputs

# Tool invocations
EBike\ Smart\ Controller.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Linux GCC C Linker'
	arm-none-eabi-gcc -nostartfiles -Wl,-Map,EBike Smart Controller.map -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "EBike Smart Controller.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

EBike\ Smart\ Controller.hex: EBike\ Smart\ Controller.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex EBike Smart Controller.elf  "EBike Smart Controller.hex"
	@echo 'Finished building: $@'
	@echo ' '

EBike\ Smart\ Controller.lst: EBike\ Smart\ Controller.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Listing'
	arm-none-eabi-objdump -h -S EBike Smart Controller.elf > "EBike Smart Controller.lst"
	@echo 'Finished building: $@'
	@echo ' '

EBike\ Smart\ Controller.siz: EBike\ Smart\ Controller.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Print Size'
	arm-none-eabi-size  --format=berkeley EBike Smart Controller.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) "EBike Smart Controller.elf"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
