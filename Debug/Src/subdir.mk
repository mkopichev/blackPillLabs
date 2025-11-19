################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/changeLabButton.c \
../Src/common.c \
../Src/lab1LedStrip.c \
../Src/lab2PwmAdc.c \
../Src/lab3UartAdcDma.c \
../Src/main.c \
../Src/multiFuncButton.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/uart.c 

OBJS += \
./Src/changeLabButton.o \
./Src/common.o \
./Src/lab1LedStrip.o \
./Src/lab2PwmAdc.o \
./Src/lab3UartAdcDma.o \
./Src/main.o \
./Src/multiFuncButton.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/uart.o 

C_DEPS += \
./Src/changeLabButton.d \
./Src/common.d \
./Src/lab1LedStrip.d \
./Src/lab2PwmAdc.d \
./Src/lab3UartAdcDma.d \
./Src/main.d \
./Src/multiFuncButton.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401CCUx -DSTM32 -DSTM32F4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/changeLabButton.cyclo ./Src/changeLabButton.d ./Src/changeLabButton.o ./Src/changeLabButton.su ./Src/common.cyclo ./Src/common.d ./Src/common.o ./Src/common.su ./Src/lab1LedStrip.cyclo ./Src/lab1LedStrip.d ./Src/lab1LedStrip.o ./Src/lab1LedStrip.su ./Src/lab2PwmAdc.cyclo ./Src/lab2PwmAdc.d ./Src/lab2PwmAdc.o ./Src/lab2PwmAdc.su ./Src/lab3UartAdcDma.cyclo ./Src/lab3UartAdcDma.d ./Src/lab3UartAdcDma.o ./Src/lab3UartAdcDma.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/multiFuncButton.cyclo ./Src/multiFuncButton.d ./Src/multiFuncButton.o ./Src/multiFuncButton.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

