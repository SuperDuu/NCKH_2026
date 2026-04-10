################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ILI9341.c \
../Core/Src/LED.c \
../Core/Src/LOGO.c \
../Core/Src/OV7670.c \
../Core/Src/bno055.c \
../Core/Src/button.c \
../Core/Src/main.c \
../Core/Src/pca9685.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/w9825g6kh.c 

OBJS += \
./Core/Src/ILI9341.o \
./Core/Src/LED.o \
./Core/Src/LOGO.o \
./Core/Src/OV7670.o \
./Core/Src/bno055.o \
./Core/Src/button.o \
./Core/Src/main.o \
./Core/Src/pca9685.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/w9825g6kh.o 

C_DEPS += \
./Core/Src/ILI9341.d \
./Core/Src/LED.d \
./Core/Src/LOGO.d \
./Core/Src/OV7670.d \
./Core/Src/bno055.d \
./Core/Src/button.d \
./Core/Src/main.d \
./Core/Src/pca9685.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/w9825g6kh.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../X-CUBE-AI -I../X-CUBE-AI/App -I../Middlewares/ST/AI/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ILI9341.cyclo ./Core/Src/ILI9341.d ./Core/Src/ILI9341.o ./Core/Src/ILI9341.su ./Core/Src/LED.cyclo ./Core/Src/LED.d ./Core/Src/LED.o ./Core/Src/LED.su ./Core/Src/LOGO.cyclo ./Core/Src/LOGO.d ./Core/Src/LOGO.o ./Core/Src/LOGO.su ./Core/Src/OV7670.cyclo ./Core/Src/OV7670.d ./Core/Src/OV7670.o ./Core/Src/OV7670.su ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/button.cyclo ./Core/Src/button.d ./Core/Src/button.o ./Core/Src/button.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pca9685.cyclo ./Core/Src/pca9685.d ./Core/Src/pca9685.o ./Core/Src/pca9685.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/w9825g6kh.cyclo ./Core/Src/w9825g6kh.d ./Core/Src/w9825g6kh.o ./Core/Src/w9825g6kh.su

.PHONY: clean-Core-2f-Src

