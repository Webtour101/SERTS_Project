################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := clean

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include RTE/USB/subdir.mk
-include RTE/File_System/subdir.mk
-include RTE/Device/STM32F407VGTx/subdir.mk
-include RTE/Compiler/subdir.mk
-include RTE/CMSIS_Driver/subdir.mk
-include RTE/CMSIS/subdir.mk
-include RTE/Board_Support/subdir.mk
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
all: Project.axf

# Tool invocations
Project.axf: $(OBJS) C:\Users\Josiah\ Ferguson\Development\ Studio\ Workspace\Project/cmsis_rte_STM32F407VGTx.sct $(USER_OBJS) $(LIBS)
	@echo 'Building target: $@'
	@echo 'Invoking: Arm Linker 6'
	armlink.exe --entry=Reset_Handler --scatter="C:\Users\Josiah Ferguson\Development Studio Workspace\Project/cmsis_rte_STM32F407VGTx.sct" --info=sizes -o "Project.axf" $(OBJS) $(USER_OBJS) "C:/Users/Josiah Ferguson/AppData/Local/Arm/Packs/ARM/CMSIS/5.8.0/CMSIS/RTOS/RTX/LIB/ARM/RTX_CM4.lib" "C:/Users/Josiah Ferguson/AppData/Local/Arm/Packs/Keil/MDK-Middleware/7.13.0/FileSystem/Lib/ARM/FS_LFN_CM3_L.lib" "C:/Users/Josiah Ferguson/AppData/Local/Arm/Packs/Keil/MDK-Middleware/7.13.0/USB/Lib/ARM/USB_CM3_L.lib" "C:/Users/Josiah Ferguson/Development Studio Workspace/Project/src/libPDMFilter_CM4_Keil.lib"
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES) $(OBJS) $(C_DEPS) Project.axf 
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
