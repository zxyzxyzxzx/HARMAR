################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../startup/cr_startup_lpc12xx.c 

S_SRCS += \
../startup/aeabi_romdiv_patch.s 

OBJS += \
./startup/aeabi_romdiv_patch.o \
./startup/cr_startup_lpc12xx.o 

C_DEPS += \
./startup/cr_startup_lpc12xx.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -DNDEBUG -D__CODE_RED -D__USE_ROMDIVIDE -mcpu=cortex-m0 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -D__USE_CMSIS=CMSISv2p00_LPC12xx -D__DISABLE_WATCHDOG -D__USE_ROMDIVIDE -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\CMSISv2p00_LPC12xx\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\lib_small_printf_m0\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\UAB_COMMON_LIB\inc" -I"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\UAB_HELIX\inc" -Os -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


