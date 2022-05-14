################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/list.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

C_DEPS += \
./Middlewares/FreeRTOS/cmsis_os2.d \
./Middlewares/FreeRTOS/croutine.d \
./Middlewares/FreeRTOS/event_groups.d \
./Middlewares/FreeRTOS/heap_4.d \
./Middlewares/FreeRTOS/list.d \
./Middlewares/FreeRTOS/port.d \
./Middlewares/FreeRTOS/queue.d \
./Middlewares/FreeRTOS/stream_buffer.d \
./Middlewares/FreeRTOS/tasks.d \
./Middlewares/FreeRTOS/timers.d 

OBJS += \
./Middlewares/FreeRTOS/cmsis_os2.o \
./Middlewares/FreeRTOS/croutine.o \
./Middlewares/FreeRTOS/event_groups.o \
./Middlewares/FreeRTOS/heap_4.o \
./Middlewares/FreeRTOS/list.o \
./Middlewares/FreeRTOS/port.o \
./Middlewares/FreeRTOS/queue.o \
./Middlewares/FreeRTOS/stream_buffer.o \
./Middlewares/FreeRTOS/tasks.o \
./Middlewares/FreeRTOS/timers.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/cmsis_os2.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/croutine.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/event_groups.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/heap_4.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/list.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/list.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/port.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/queue.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/stream_buffer.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/tasks.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/timers.o: C:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/molso/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.1/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FreeRTOS

clean-Middlewares-2f-FreeRTOS:
	-$(RM) ./Middlewares/FreeRTOS/cmsis_os2.d ./Middlewares/FreeRTOS/cmsis_os2.o ./Middlewares/FreeRTOS/croutine.d ./Middlewares/FreeRTOS/croutine.o ./Middlewares/FreeRTOS/event_groups.d ./Middlewares/FreeRTOS/event_groups.o ./Middlewares/FreeRTOS/heap_4.d ./Middlewares/FreeRTOS/heap_4.o ./Middlewares/FreeRTOS/list.d ./Middlewares/FreeRTOS/list.o ./Middlewares/FreeRTOS/port.d ./Middlewares/FreeRTOS/port.o ./Middlewares/FreeRTOS/queue.d ./Middlewares/FreeRTOS/queue.o ./Middlewares/FreeRTOS/stream_buffer.d ./Middlewares/FreeRTOS/stream_buffer.o ./Middlewares/FreeRTOS/tasks.d ./Middlewares/FreeRTOS/tasks.o ./Middlewares/FreeRTOS/timers.d ./Middlewares/FreeRTOS/timers.o

.PHONY: clean-Middlewares-2f-FreeRTOS

