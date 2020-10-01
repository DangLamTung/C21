################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
C21_FlappyBird.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
C21_FlappyBird.list \

OBJCOPY_BIN += \
C21_FlappyBird.bin \


# All Target
all: C21_FlappyBird.elf secondary-outputs

# Tool invocations
C21_FlappyBird.elf: $(OBJS) $(USER_OBJS) C:\Users\UAD81HC\Documents\GitHub\C21\C21_Software\C21_FlappyBird\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "C21_FlappyBird.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\UAD81HC\Documents\GitHub\C21\C21_Software\C21_FlappyBird\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="C21_FlappyBird.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

C21_FlappyBird.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "C21_FlappyBird.list"
	@echo 'Finished building: $@'
	@echo ' '

C21_FlappyBird.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "C21_FlappyBird.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
