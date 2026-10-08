################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Extend/Font/OLED_Font.c 

OBJS += \
./Core/Extend/Font/OLED_Font.o 

C_DEPS += \
./Core/Extend/Font/OLED_Font.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Extend/Font/%.o Core/Extend/Font/%.su Core/Extend/Font/%.cyclo: ../Core/Extend/Font/%.c Core/Extend/Font/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Extend/Font -I../Core/Extend/HeadFiel -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Extend-2f-Font

clean-Core-2f-Extend-2f-Font:
	-$(RM) ./Core/Extend/Font/OLED_Font.cyclo ./Core/Extend/Font/OLED_Font.d ./Core/Extend/Font/OLED_Font.o ./Core/Extend/Font/OLED_Font.su

.PHONY: clean-Core-2f-Extend-2f-Font

