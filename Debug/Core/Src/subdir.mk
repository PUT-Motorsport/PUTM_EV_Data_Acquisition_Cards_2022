################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/ism330dhcx_reg.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_hal_timebase_tim.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

CPP_SRCS += \
../Core/Src/ISM330DHCXSensor.cpp \
../Core/Src/adc_data.cpp \
../Core/Src/can_functions.cpp \
../Core/Src/ism_task.cpp \
../Core/Src/main.cpp \
../Core/Src/meansure_task.cpp 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/ism330dhcx_reg.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_hal_timebase_tim.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 

OBJS += \
./Core/Src/ISM330DHCXSensor.o \
./Core/Src/adc_data.o \
./Core/Src/can_functions.o \
./Core/Src/freertos.o \
./Core/Src/ism330dhcx_reg.o \
./Core/Src/ism_task.o \
./Core/Src/main.o \
./Core/Src/meansure_task.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_hal_timebase_tim.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

CPP_DEPS += \
./Core/Src/ISM330DHCXSensor.d \
./Core/Src/adc_data.d \
./Core/Src/can_functions.d \
./Core/Src/ism_task.d \
./Core/Src/main.d \
./Core/Src/meansure_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ISM330DHCXSensor.d ./Core/Src/ISM330DHCXSensor.o ./Core/Src/adc_data.d ./Core/Src/adc_data.o ./Core/Src/can_functions.d ./Core/Src/can_functions.o ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/ism330dhcx_reg.d ./Core/Src/ism330dhcx_reg.o ./Core/Src/ism_task.d ./Core/Src/ism_task.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/meansure_task.d ./Core/Src/meansure_task.o ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_timebase_tim.d ./Core/Src/stm32l4xx_hal_timebase_tim.o ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o

.PHONY: clean-Core-2f-Src

