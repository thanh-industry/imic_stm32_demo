################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utils/SEGGER_RTT.c \
../Utils/SEGGER_RTT_printf.c \
../Utils/led.c 

OBJS += \
./Utils/SEGGER_RTT.o \
./Utils/SEGGER_RTT_printf.o \
./Utils/led.o 

C_DEPS += \
./Utils/SEGGER_RTT.d \
./Utils/SEGGER_RTT_printf.d \
./Utils/led.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/%.o Utils/%.su Utils/%.cyclo: ../Utils/%.c Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -I../Drivers/Device -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../LWIP/App -I../LWIP/Target -I"D:/IMIC/stm32_rfid_scanner/Middlewares/Third_Party/LwIP/src/include" -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utils

clean-Utils:
	-$(RM) ./Utils/SEGGER_RTT.cyclo ./Utils/SEGGER_RTT.d ./Utils/SEGGER_RTT.o ./Utils/SEGGER_RTT.su ./Utils/SEGGER_RTT_printf.cyclo ./Utils/SEGGER_RTT_printf.d ./Utils/SEGGER_RTT_printf.o ./Utils/SEGGER_RTT_printf.su ./Utils/led.cyclo ./Utils/led.d ./Utils/led.o ./Utils/led.su

.PHONY: clean-Utils

