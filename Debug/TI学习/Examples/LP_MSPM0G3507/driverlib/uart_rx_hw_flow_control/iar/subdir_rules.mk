################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
TI学习/Examples/LP_MSPM0G3507/driverlib/uart_rx_hw_flow_control/iar/%.o: ../TI学习/Examples/LP_MSPM0G3507/driverlib/uart_rx_hw_flow_control/iar/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Vico/Desktop/26NUEDC/Code/main/NUEDC2026" -I"C:/Users/Vico/Desktop/26NUEDC/Code/main/NUEDC2026/Debug" -I"C:/ti/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"TI学习/Examples/LP_MSPM0G3507/driverlib/uart_rx_hw_flow_control/iar/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"


