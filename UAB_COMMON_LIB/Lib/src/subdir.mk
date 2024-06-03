################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/alarms.c \
../src/compadc.c \
../src/iap.c \
../src/ir.c \
../src/speed.c \
../src/uart.c \
../src/utils.c \
../src/wdt.c 

OBJS += \
./src/alarms.o \
./src/compadc.o \
./src/iap.o \
./src/ir.o \
./src/speed.o \
./src/uart.o \
./src/utils.o \
./src/wdt.o 

C_DEPS += \
./src/alarms.d \
./src/compadc.d \
./src/iap.d \
./src/ir.d \
./src/speed.d \
./src/uart.d \
./src/utils.d \
./src/wdt.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -D__REDLIB__ -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\CMSISv2p00_LPC12xx\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\lib_small_printf_m0\inc" -I../inc -O0 -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


