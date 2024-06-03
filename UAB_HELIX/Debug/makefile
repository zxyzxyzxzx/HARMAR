################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: UAB_HELIX.axf

# Tool invocations
UAB_HELIX.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\CMSISv2p00_LPC12xx\Debug" -L"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\lib_small_printf_m0\Lib" -L"C:\Users\Zhang\OneDrive\SUNS\Project\Harmar\Work\UAB_COMMON_LIB\Lib" -Xlinker -Map="UAB_HELIX.map" -Xlinker --gc-sections -Xlinker --allow-multiple-definition -mcpu=cortex-m0 -mthumb -T "UAB_HELIX_Debug.ld" -o "UAB_HELIX.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) UAB_HELIX.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "UAB_HELIX.axf"; arm-none-eabi-objcopy -O binary "UAB_HELIX.axf" "UAB_HELIX.bin" ; checksum -p LPC1225_301 -d "UAB_HELIX.bin"; arm-none-eabi-objcopy -I binary -O ihex "UAB_HELIX.bin"  "UAB_HELIX.hex";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
