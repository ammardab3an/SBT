################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f091rctx.s 

S_DEPS += \
./Core/Startup/startup_stm32f091rctx.d 

OBJS += \
./Core/Startup/startup_stm32f091rctx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -DDEBUG -DMPL_LOG_NDEBUG=1 -DMPU6050 -DEMPL -DUSE_DMP -DEMPL_TARGET_STM32F4 -c -I"C:/Users/AmmarDab3an/STM32CubeIDE/workspace_1.13.2/selfBalancingBoard/MPU" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f091rctx.d ./Core/Startup/startup_stm32f091rctx.o

.PHONY: clean-Core-2f-Startup

