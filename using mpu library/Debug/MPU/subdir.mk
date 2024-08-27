################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU/data_builder.c \
../MPU/eMPL_outputs.c \
../MPU/hal_outputs.c \
../MPU/inv_mpu.c \
../MPU/inv_mpu_dmp_motion_driver.c \
../MPU/log_stm32.c \
../MPU/message_layer.c \
../MPU/ml_math_func.c \
../MPU/mlmath.c \
../MPU/mpl.c \
../MPU/results_holder.c \
../MPU/start_manager.c \
../MPU/storage_manager.c 

C_DEPS += \
./MPU/data_builder.d \
./MPU/eMPL_outputs.d \
./MPU/hal_outputs.d \
./MPU/inv_mpu.d \
./MPU/inv_mpu_dmp_motion_driver.d \
./MPU/log_stm32.d \
./MPU/message_layer.d \
./MPU/ml_math_func.d \
./MPU/mlmath.d \
./MPU/mpl.d \
./MPU/results_holder.d \
./MPU/start_manager.d \
./MPU/storage_manager.d 

OBJS += \
./MPU/data_builder.o \
./MPU/eMPL_outputs.o \
./MPU/hal_outputs.o \
./MPU/inv_mpu.o \
./MPU/inv_mpu_dmp_motion_driver.o \
./MPU/log_stm32.o \
./MPU/message_layer.o \
./MPU/ml_math_func.o \
./MPU/mlmath.o \
./MPU/mpl.o \
./MPU/results_holder.o \
./MPU/start_manager.o \
./MPU/storage_manager.o 


# Each subdirectory must supply rules for building sources it contributes
MPU/%.o MPU/%.su MPU/%.cyclo: ../MPU/%.c MPU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -DMPL_LOG_NDEBUG=1 -DMPU6050 -DEMPL -DUSE_DMP -DEMPL_TARGET_STM32F4 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/AmmarDab3an/STM32CubeIDE/workspace_1.13.2/selfBalancingBoard/MPU" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MPU

clean-MPU:
	-$(RM) ./MPU/data_builder.cyclo ./MPU/data_builder.d ./MPU/data_builder.o ./MPU/data_builder.su ./MPU/eMPL_outputs.cyclo ./MPU/eMPL_outputs.d ./MPU/eMPL_outputs.o ./MPU/eMPL_outputs.su ./MPU/hal_outputs.cyclo ./MPU/hal_outputs.d ./MPU/hal_outputs.o ./MPU/hal_outputs.su ./MPU/inv_mpu.cyclo ./MPU/inv_mpu.d ./MPU/inv_mpu.o ./MPU/inv_mpu.su ./MPU/inv_mpu_dmp_motion_driver.cyclo ./MPU/inv_mpu_dmp_motion_driver.d ./MPU/inv_mpu_dmp_motion_driver.o ./MPU/inv_mpu_dmp_motion_driver.su ./MPU/log_stm32.cyclo ./MPU/log_stm32.d ./MPU/log_stm32.o ./MPU/log_stm32.su ./MPU/message_layer.cyclo ./MPU/message_layer.d ./MPU/message_layer.o ./MPU/message_layer.su ./MPU/ml_math_func.cyclo ./MPU/ml_math_func.d ./MPU/ml_math_func.o ./MPU/ml_math_func.su ./MPU/mlmath.cyclo ./MPU/mlmath.d ./MPU/mlmath.o ./MPU/mlmath.su ./MPU/mpl.cyclo ./MPU/mpl.d ./MPU/mpl.o ./MPU/mpl.su ./MPU/results_holder.cyclo ./MPU/results_holder.d ./MPU/results_holder.o ./MPU/results_holder.su ./MPU/start_manager.cyclo ./MPU/start_manager.d ./MPU/start_manager.o ./MPU/start_manager.su ./MPU/storage_manager.cyclo ./MPU/storage_manager.d ./MPU/storage_manager.o ./MPU/storage_manager.su

.PHONY: clean-MPU

