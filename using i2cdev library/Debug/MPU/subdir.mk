################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MPU/ArduinoWrapper.cpp \
../MPU/I2Cdev.cpp \
../MPU/MPU6050.cpp 

OBJS += \
./MPU/ArduinoWrapper.o \
./MPU/I2Cdev.o \
./MPU/MPU6050.o 

CPP_DEPS += \
./MPU/ArduinoWrapper.d \
./MPU/I2Cdev.d \
./MPU/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
MPU/ArduinoWrapper.o: ../MPU/ArduinoWrapper.cpp MPU/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../MPU -O2 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MPU/ArduinoWrapper.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MPU/I2Cdev.o: ../MPU/I2Cdev.cpp MPU/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../MPU -O2 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MPU/I2Cdev.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MPU/MPU6050.o: ../MPU/MPU6050.cpp MPU/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../MPU -O2 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MPU/MPU6050.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

