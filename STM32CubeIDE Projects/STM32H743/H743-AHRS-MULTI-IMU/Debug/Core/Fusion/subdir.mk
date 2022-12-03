################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Fusion/FusionAhrs.c \
../Core/Fusion/FusionBias.c \
../Core/Fusion/FusionCompass.c 

OBJS += \
./Core/Fusion/FusionAhrs.o \
./Core/Fusion/FusionBias.o \
./Core/Fusion/FusionCompass.o 

C_DEPS += \
./Core/Fusion/FusionAhrs.d \
./Core/Fusion/FusionBias.d \
./Core/Fusion/FusionCompass.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Fusion/%.o Core/Fusion/%.su: ../Core/Fusion/%.c Core/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/ibrahim/Desktop/VTVL-CORE/STM32CubeIDE Projects/STM32H743/H743-AHRS-MULTI-IMU/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Fusion

clean-Core-2f-Fusion:
	-$(RM) ./Core/Fusion/FusionAhrs.d ./Core/Fusion/FusionAhrs.o ./Core/Fusion/FusionAhrs.su ./Core/Fusion/FusionBias.d ./Core/Fusion/FusionBias.o ./Core/Fusion/FusionBias.su ./Core/Fusion/FusionCompass.d ./Core/Fusion/FusionCompass.o ./Core/Fusion/FusionCompass.su

.PHONY: clean-Core-2f-Fusion

