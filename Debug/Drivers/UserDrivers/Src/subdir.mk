################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UserDrivers/Src/usart.c 

OBJS += \
./Drivers/UserDrivers/Src/usart.o 

C_DEPS += \
./Drivers/UserDrivers/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UserDrivers/Src/%.o: ../Drivers/UserDrivers/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F334x8 -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Core/Inc" -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Drivers/CMSIS/Include" -I"C:/Users/damian/Documents/_Desarrollo/USART con DMA y interrupcion IDLE/USART_idle_dma/Drivers/UserDrivers/Inc"  -O3 -g3 -Wall -fmessage-length=0 -u _printf_float -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


