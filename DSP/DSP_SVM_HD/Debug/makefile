################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/TI/ccsv6/tools/compiler/c2000_15.12.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./source/39vf1601.obj" \
"./source/DSP2834x_CodeStartBranch.obj" \
"./source/DSP2834x_CpuTimers.obj" \
"./source/DSP2834x_DMA.obj" \
"./source/DSP2834x_DefaultIsr.obj" \
"./source/DSP2834x_GlobalVariableDefs.obj" \
"./source/DSP2834x_MemCopy.obj" \
"./source/DSP2834x_PieCtrl.obj" \
"./source/DSP2834x_PieVect.obj" \
"./source/DSP2834x_SysCtrl.obj" \
"./source/DSP2834x_Xintf.obj" \
"./source/DSP2834x_usDelay.obj" \
"./source/Sintable.obj" \
"./source/main.obj" \
"../CMD/28346_RAM_lnk.cmd" \
"../CMD/DSP2834x_Headers_nonBIOS.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include CMD/subdir_vars.mk
-include source/subdir_vars.mk
-include CMD/subdir_rules.mk
-include source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
SVM_HD.out \

EXE_OUTPUTS__QUOTED += \
"SVM_HD.out" \

BIN_OUTPUTS += \
SVM_HD.hex \

BIN_OUTPUTS__QUOTED += \
"SVM_HD.hex" \


# All Target
all: SVM_HD.out

# Tool invocations
SVM_HD.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"D:/TI/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --display_error_number --diag_warning=225 --diag_wrap=off -z -m"SVM_HD.map" --stack_size=0x300 --warn_sections -i"D:/TI/ccsv6/tools/compiler/c2000_15.12.3.LTS/lib" -i"D:/TI/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="SVM_HD_linkInfo.xml" --rom_model -o "SVM_HD.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

SVM_HD.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"D:/TI/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/hex2000"  -o "SVM_HD.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "source\39vf1601.d" "source\DSP2834x_CpuTimers.d" "source\DSP2834x_DMA.d" "source\DSP2834x_DefaultIsr.d" "source\DSP2834x_GlobalVariableDefs.d" "source\DSP2834x_MemCopy.d" "source\DSP2834x_PieCtrl.d" "source\DSP2834x_PieVect.d" "source\DSP2834x_SysCtrl.d" "source\DSP2834x_Xintf.d" "source\Sintable.d" "source\main.d" 
	-$(RM) "source\39vf1601.obj" "source\DSP2834x_CodeStartBranch.obj" "source\DSP2834x_CpuTimers.obj" "source\DSP2834x_DMA.obj" "source\DSP2834x_DefaultIsr.obj" "source\DSP2834x_GlobalVariableDefs.obj" "source\DSP2834x_MemCopy.obj" "source\DSP2834x_PieCtrl.obj" "source\DSP2834x_PieVect.obj" "source\DSP2834x_SysCtrl.obj" "source\DSP2834x_Xintf.obj" "source\DSP2834x_usDelay.obj" "source\Sintable.obj" "source\main.obj" 
	-$(RM) "source\DSP2834x_CodeStartBranch.d" "source\DSP2834x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

