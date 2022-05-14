################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/utility/spi_com.c 

C_DEPS += \
./Core/Inc/utility/spi_com.d 

OBJS += \
./Core/Inc/utility/spi_com.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/utility/%.o: ../Core/Inc/utility/%.c Core/Inc/utility/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-utility

clean-Core-2f-Inc-2f-utility:
	-$(RM) ./Core/Inc/utility/spi_com.d ./Core/Inc/utility/spi_com.o

.PHONY: clean-Core-2f-Inc-2f-utility

