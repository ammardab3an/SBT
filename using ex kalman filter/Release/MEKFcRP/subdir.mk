################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MEKFcRP/mpu6050.cpp 

OBJS += \
./MEKFcRP/mpu6050.o 

CPP_DEPS += \
./MEKFcRP/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
MEKFcRP/mpu6050.o: ../MEKFcRP/mpu6050.cpp MEKFcRP/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++17 -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/AmmarDab3an/STM32CubeIDE/workspace_1.6.1/test/MEKFcRP" -Ofast -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MEKFcRP/mpu6050.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

