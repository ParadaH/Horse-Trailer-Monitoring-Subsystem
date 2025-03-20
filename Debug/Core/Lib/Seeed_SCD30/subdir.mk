################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Lib/Seeed_SCD30/SCD30.cpp 

OBJS += \
./Core/Lib/Seeed_SCD30/SCD30.o 

CPP_DEPS += \
./Core/Lib/Seeed_SCD30/SCD30.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/Seeed_SCD30/%.o Core/Lib/Seeed_SCD30/%.su Core/Lib/Seeed_SCD30/%.cyclo: ../Core/Lib/Seeed_SCD30/%.cpp Core/Lib/Seeed_SCD30/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -std=c++17 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-Seeed_SCD30

clean-Core-2f-Lib-2f-Seeed_SCD30:
	-$(RM) ./Core/Lib/Seeed_SCD30/SCD30.cyclo ./Core/Lib/Seeed_SCD30/SCD30.d ./Core/Lib/Seeed_SCD30/SCD30.o ./Core/Lib/Seeed_SCD30/SCD30.su

.PHONY: clean-Core-2f-Lib-2f-Seeed_SCD30

