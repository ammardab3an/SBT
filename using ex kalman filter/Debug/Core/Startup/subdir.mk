################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f091rctx.s 

S_DEPS += \
./Core/Startup/startup_stm32f091rctx.d 

OBJS += \
./Core/Startup/startup_stm32f091rctx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f091rctx.o: ../Core/Startup/startup_stm32f091rctx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g -c -I"C:/Users/AmmarDab3an/STM32CubeIDE/workspace_1.6.1/test/MEKFcRP" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f091rctx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

