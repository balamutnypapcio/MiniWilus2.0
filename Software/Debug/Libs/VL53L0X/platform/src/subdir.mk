################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libs/VL53L0X/platform/src/vl53l0x_platform.c \
../Libs/VL53L0X/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Libs/VL53L0X/platform/src/vl53l0x_platform.o \
./Libs/VL53L0X/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Libs/VL53L0X/platform/src/vl53l0x_platform.d \
./Libs/VL53L0X/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/VL53L0X/platform/src/%.o Libs/VL53L0X/platform/src/%.su Libs/VL53L0X/platform/src/%.cyclo: ../Libs/VL53L0X/platform/src/%.c Libs/VL53L0X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I"/home/jakub/Projekty/MiniWilus2.0/Software/Libs/VL53L0X/core/inc" -I"/home/jakub/Projekty/MiniWilus2.0/Software/Libs/VL53L0X/platform/inc" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libs-2f-VL53L0X-2f-platform-2f-src

clean-Libs-2f-VL53L0X-2f-platform-2f-src:
	-$(RM) ./Libs/VL53L0X/platform/src/vl53l0x_platform.cyclo ./Libs/VL53L0X/platform/src/vl53l0x_platform.d ./Libs/VL53L0X/platform/src/vl53l0x_platform.o ./Libs/VL53L0X/platform/src/vl53l0x_platform.su ./Libs/VL53L0X/platform/src/vl53l0x_platform_log.cyclo ./Libs/VL53L0X/platform/src/vl53l0x_platform_log.d ./Libs/VL53L0X/platform/src/vl53l0x_platform_log.o ./Libs/VL53L0X/platform/src/vl53l0x_platform_log.su

.PHONY: clean-Libs-2f-VL53L0X-2f-platform-2f-src

