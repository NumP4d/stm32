################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Drivers/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
sound_encoder_fft.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
sound_encoder_fft.list \


# All Target
all: sound_encoder_fft.elf secondary-outputs

# Tool invocations
sound_encoder_fft.elf: $(OBJS) $(USER_OBJS) /home/klukomski/workspace/stm32/sound_encoder_fft/STM32L496AGIX_FLASH.ld
	arm-none-eabi-gcc -o "sound_encoder_fft.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/klukomski/workspace/stm32/sound_encoder_fft/STM32L496AGIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="sound_encoder_fft.map" -Wl,--gc-sections -static -L"/home/klukomski/workspace/stm32/CMSIS/CMSIS/Lib/GCC" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

sound_encoder_fft.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "sound_encoder_fft.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
