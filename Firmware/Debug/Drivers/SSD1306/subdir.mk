################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306/ssd1306.c \
../Drivers/SSD1306/ssd1306_fonts.c 

OBJS += \
./Drivers/SSD1306/ssd1306.o \
./Drivers/SSD1306/ssd1306_fonts.o 

C_DEPS += \
./Drivers/SSD1306/ssd1306.d \
./Drivers/SSD1306/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306/%.o Drivers/SSD1306/%.su Drivers/SSD1306/%.cyclo: ../Drivers/SSD1306/%.c Drivers/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I"E:/Privat/Git/DICV3/Firmware/Drivers/SSD1306" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"E:/Privat/Git/DICV3/Firmware/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306

clean-Drivers-2f-SSD1306:
	-$(RM) ./Drivers/SSD1306/ssd1306.cyclo ./Drivers/SSD1306/ssd1306.d ./Drivers/SSD1306/ssd1306.o ./Drivers/SSD1306/ssd1306.su ./Drivers/SSD1306/ssd1306_fonts.cyclo ./Drivers/SSD1306/ssd1306_fonts.d ./Drivers/SSD1306/ssd1306_fonts.o ./Drivers/SSD1306/ssd1306_fonts.su

.PHONY: clean-Drivers-2f-SSD1306

