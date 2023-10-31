################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (7-2018-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Device/MFRC522.c 

OBJS += \
./Drivers/Device/MFRC522.o 

C_DEPS += \
./Drivers/Device/MFRC522.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Device/%.o Drivers/Device/%.su Drivers/Device/%.cyclo: ../Drivers/Device/%.c Drivers/Device/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Device

clean-Drivers-2f-Device:
	-$(RM) ./Drivers/Device/MFRC522.cyclo ./Drivers/Device/MFRC522.d ./Drivers/Device/MFRC522.o ./Drivers/Device/MFRC522.su

.PHONY: clean-Drivers-2f-Device

