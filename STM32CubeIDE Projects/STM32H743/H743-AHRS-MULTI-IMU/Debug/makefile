################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Fusion/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := H743-AHRS-MULTI-IMU
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
H743-AHRS-MULTI-IMU.elf \

MAP_FILES += \
H743-AHRS-MULTI-IMU.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
H743-AHRS-MULTI-IMU.list \


# All Target
all: main-build

# Main-build Target
main-build: H743-AHRS-MULTI-IMU.elf secondary-outputs

# Tool invocations
H743-AHRS-MULTI-IMU.elf H743-AHRS-MULTI-IMU.map: $(OBJS) $(USER_OBJS) C:\Users\ibrahim\Desktop\VTVL\STM32CubeIDE\ Projects\STM32H743\H743-AHRS-MULTI-IMU\STM32H743VITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "H743-AHRS-MULTI-IMU.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\ibrahim\Desktop\VTVL\STM32CubeIDE Projects\STM32H743\H743-AHRS-MULTI-IMU\STM32H743VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="H743-AHRS-MULTI-IMU.map" -Wl,--gc-sections -static -L"C:\Users\ibrahim\Desktop\VTVL\STM32CubeIDE Projects\STM32H743\H743-AHRS-MULTI-IMU\Drivers\CMSIS\DSP\Lib\GCC" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

H743-AHRS-MULTI-IMU.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "H743-AHRS-MULTI-IMU.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) H743-AHRS-MULTI-IMU.elf H743-AHRS-MULTI-IMU.list H743-AHRS-MULTI-IMU.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
