################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main_PINNACLE.c \
../src/uab_PINNACLE.c 

OBJS += \
./src/main_PINNACLE.o \
./src/uab_PINNACLE.o 

C_DEPS += \
./src/main_PINNACLE.d \
./src/uab_PINNACLE.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_CMSIS=CMSISv2p00_LPC12xx -D__DISABLE_WATCHDOG -D__USE_ROMDIVIDE -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\CMSISv2p00_LPC12xx\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\lib_small_printf_m0\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\UAB_COMMON_LIB\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\UAB_PINNACLE\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


