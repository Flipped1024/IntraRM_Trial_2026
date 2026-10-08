################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Extend/C_Cpp/HAL_Add.c \
../Core/Extend/C_Cpp/MPU6050.c \
../Core/Extend/C_Cpp/OLED.c \
../Core/Extend/C_Cpp/Pin.c \
../Core/Extend/C_Cpp/filter.c 

OBJS += \
./Core/Extend/C_Cpp/HAL_Add.o \
./Core/Extend/C_Cpp/MPU6050.o \
./Core/Extend/C_Cpp/OLED.o \
./Core/Extend/C_Cpp/Pin.o \
./Core/Extend/C_Cpp/filter.o 

C_DEPS += \
./Core/Extend/C_Cpp/HAL_Add.d \
./Core/Extend/C_Cpp/MPU6050.d \
./Core/Extend/C_Cpp/OLED.d \
./Core/Extend/C_Cpp/Pin.d \
./Core/Extend/C_Cpp/filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Extend/C_Cpp/%.o Core/Extend/C_Cpp/%.su Core/Extend/C_Cpp/%.cyclo: ../Core/Extend/C_Cpp/%.c Core/Extend/C_Cpp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Extend/Font -I../Core/Extend/HeadFiel -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Extend-2f-C_Cpp

clean-Core-2f-Extend-2f-C_Cpp:
	-$(RM) ./Core/Extend/C_Cpp/HAL_Add.cyclo ./Core/Extend/C_Cpp/HAL_Add.d ./Core/Extend/C_Cpp/HAL_Add.o ./Core/Extend/C_Cpp/HAL_Add.su ./Core/Extend/C_Cpp/MPU6050.cyclo ./Core/Extend/C_Cpp/MPU6050.d ./Core/Extend/C_Cpp/MPU6050.o ./Core/Extend/C_Cpp/MPU6050.su ./Core/Extend/C_Cpp/OLED.cyclo ./Core/Extend/C_Cpp/OLED.d ./Core/Extend/C_Cpp/OLED.o ./Core/Extend/C_Cpp/OLED.su ./Core/Extend/C_Cpp/Pin.cyclo ./Core/Extend/C_Cpp/Pin.d ./Core/Extend/C_Cpp/Pin.o ./Core/Extend/C_Cpp/Pin.su ./Core/Extend/C_Cpp/filter.cyclo ./Core/Extend/C_Cpp/filter.d ./Core/Extend/C_Cpp/filter.o ./Core/Extend/C_Cpp/filter.su

.PHONY: clean-Core-2f-Extend-2f-C_Cpp

