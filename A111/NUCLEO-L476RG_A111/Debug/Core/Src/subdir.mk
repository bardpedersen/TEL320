################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.c \
../Core/Src/acc_integration_log.c \
../Core/Src/acc_integration_stm32.c \
../Core/Src/estimated_water_speed.c \
../Core/Src/main.c \
../Core/Src/peak_detection.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.o \
./Core/Src/acc_integration_log.o \
./Core/Src/acc_integration_stm32.o \
./Core/Src/estimated_water_speed.o \
./Core/Src/main.o \
./Core/Src/peak_detection.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.d \
./Core/Src/acc_integration_log.d \
./Core/Src/acc_integration_stm32.d \
./Core/Src/estimated_water_speed.d \
./Core/Src/main.d \
./Core/Src/peak_detection.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/pbuka/TEL320/TEL320/A111/NUCLEO-L476RG_A111/cortex_m4/rss/include" -I"C:/Users/pbuka/TEL320/TEL320/A111/NUCLEO-L476RG_A111/cortex_m4/integration" -I"C:/Users/pbuka/TEL320/TEL320/A111/NUCLEO-L476RG_A111/cortex_m4/examples" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.cyclo ./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.d ./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.o ./Core/Src/acc_hal_integration_stm32cube_sparkfun_a111.su ./Core/Src/acc_integration_log.cyclo ./Core/Src/acc_integration_log.d ./Core/Src/acc_integration_log.o ./Core/Src/acc_integration_log.su ./Core/Src/acc_integration_stm32.cyclo ./Core/Src/acc_integration_stm32.d ./Core/Src/acc_integration_stm32.o ./Core/Src/acc_integration_stm32.su ./Core/Src/estimated_water_speed.cyclo ./Core/Src/estimated_water_speed.d ./Core/Src/estimated_water_speed.o ./Core/Src/estimated_water_speed.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/peak_detection.cyclo ./Core/Src/peak_detection.d ./Core/Src/peak_detection.o ./Core/Src/peak_detection.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

