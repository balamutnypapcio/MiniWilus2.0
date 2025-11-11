################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libs/VL53L0X/core/src/vl53l0x_api.c \
../Libs/VL53L0X/core/src/vl53l0x_api_calibration.c \
../Libs/VL53L0X/core/src/vl53l0x_api_core.c \
../Libs/VL53L0X/core/src/vl53l0x_api_ranging.c \
../Libs/VL53L0X/core/src/vl53l0x_api_strings.c 

OBJS += \
./Libs/VL53L0X/core/src/vl53l0x_api.o \
./Libs/VL53L0X/core/src/vl53l0x_api_calibration.o \
./Libs/VL53L0X/core/src/vl53l0x_api_core.o \
./Libs/VL53L0X/core/src/vl53l0x_api_ranging.o \
./Libs/VL53L0X/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Libs/VL53L0X/core/src/vl53l0x_api.d \
./Libs/VL53L0X/core/src/vl53l0x_api_calibration.d \
./Libs/VL53L0X/core/src/vl53l0x_api_core.d \
./Libs/VL53L0X/core/src/vl53l0x_api_ranging.d \
./Libs/VL53L0X/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/VL53L0X/core/src/%.o Libs/VL53L0X/core/src/%.su Libs/VL53L0X/core/src/%.cyclo: ../Libs/VL53L0X/core/src/%.c Libs/VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I"/home/jakub/Projekty/MiniWilus2.0/Software/Libs/VL53L0X/core/inc" -I"/home/jakub/Projekty/MiniWilus2.0/Software/Libs/VL53L0X/platform/inc" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libs-2f-VL53L0X-2f-core-2f-src

clean-Libs-2f-VL53L0X-2f-core-2f-src:
	-$(RM) ./Libs/VL53L0X/core/src/vl53l0x_api.cyclo ./Libs/VL53L0X/core/src/vl53l0x_api.d ./Libs/VL53L0X/core/src/vl53l0x_api.o ./Libs/VL53L0X/core/src/vl53l0x_api.su ./Libs/VL53L0X/core/src/vl53l0x_api_calibration.cyclo ./Libs/VL53L0X/core/src/vl53l0x_api_calibration.d ./Libs/VL53L0X/core/src/vl53l0x_api_calibration.o ./Libs/VL53L0X/core/src/vl53l0x_api_calibration.su ./Libs/VL53L0X/core/src/vl53l0x_api_core.cyclo ./Libs/VL53L0X/core/src/vl53l0x_api_core.d ./Libs/VL53L0X/core/src/vl53l0x_api_core.o ./Libs/VL53L0X/core/src/vl53l0x_api_core.su ./Libs/VL53L0X/core/src/vl53l0x_api_ranging.cyclo ./Libs/VL53L0X/core/src/vl53l0x_api_ranging.d ./Libs/VL53L0X/core/src/vl53l0x_api_ranging.o ./Libs/VL53L0X/core/src/vl53l0x_api_ranging.su ./Libs/VL53L0X/core/src/vl53l0x_api_strings.cyclo ./Libs/VL53L0X/core/src/vl53l0x_api_strings.d ./Libs/VL53L0X/core/src/vl53l0x_api_strings.o ./Libs/VL53L0X/core/src/vl53l0x_api_strings.su

.PHONY: clean-Libs-2f-VL53L0X-2f-core-2f-src

