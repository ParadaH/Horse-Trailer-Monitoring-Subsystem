################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.cpp 

OBJS += \
./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.o 

CPP_DEPS += \
./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/DFRobot_OxygenSensor/src/%.o Core/Lib/DFRobot_OxygenSensor/src/%.su Core/Lib/DFRobot_OxygenSensor/src/%.cyclo: ../Core/Lib/DFRobot_OxygenSensor/src/%.cpp Core/Lib/DFRobot_OxygenSensor/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-DFRobot_OxygenSensor-2f-src

clean-Core-2f-Lib-2f-DFRobot_OxygenSensor-2f-src:
	-$(RM) ./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.cyclo ./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.d ./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.o ./Core/Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.su

.PHONY: clean-Core-2f-Lib-2f-DFRobot_OxygenSensor-2f-src

