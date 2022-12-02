################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FusionAHRS.c \
../Core/Src/LowPassFilter.c \
../Core/Src/NotchFilter.c \
../Core/Src/bmi160.c \
../Core/Src/bmi160_wrapper.c \
../Core/Src/common_porting.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/quaternion.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/FusionAHRS.o \
./Core/Src/LowPassFilter.o \
./Core/Src/NotchFilter.o \
./Core/Src/bmi160.o \
./Core/Src/bmi160_wrapper.o \
./Core/Src/common_porting.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/quaternion.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/FusionAHRS.d \
./Core/Src/LowPassFilter.d \
./Core/Src/NotchFilter.d \
./Core/Src/bmi160.d \
./Core/Src/bmi160_wrapper.d \
./Core/Src/common_porting.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/quaternion.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/ibrahim/Desktop/VTVL/STM32CubeIDE Projects/STM32H743/H743-AHRS-BMI160/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FusionAHRS.d ./Core/Src/FusionAHRS.o ./Core/Src/FusionAHRS.su ./Core/Src/LowPassFilter.d ./Core/Src/LowPassFilter.o ./Core/Src/LowPassFilter.su ./Core/Src/NotchFilter.d ./Core/Src/NotchFilter.o ./Core/Src/NotchFilter.su ./Core/Src/bmi160.d ./Core/Src/bmi160.o ./Core/Src/bmi160.su ./Core/Src/bmi160_wrapper.d ./Core/Src/bmi160_wrapper.o ./Core/Src/bmi160_wrapper.su ./Core/Src/common_porting.d ./Core/Src/common_porting.o ./Core/Src/common_porting.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

