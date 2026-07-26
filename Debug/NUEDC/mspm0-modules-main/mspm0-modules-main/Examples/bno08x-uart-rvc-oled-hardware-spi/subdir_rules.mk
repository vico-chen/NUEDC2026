################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1854964029: ../NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/bno08x-uart-rvc-oled-hardware-spi.syscfg
	@echo 'SysConfig: "$<"'
	"C:/ti/ccs2100/ccs/utils/sysconfig_1.28.0/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_2_11_00_07/.metadata/product.json" -s "C:/ti/mspm0_sdk_2_11_00_07/.metadata/product.json" --script "C:/Users/Vico/Desktop/26NUEDC/Code/main/NUEDC2026/NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/bno08x-uart-rvc-oled-hardware-spi.syscfg" -o "." --compiler ticlang

device_linker.cmd: build-1854964029 ../NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/bno08x-uart-rvc-oled-hardware-spi.syscfg
device.opt: build-1854964029
device.cmd.genlibs: build-1854964029
ti_msp_dl_config.c: build-1854964029
ti_msp_dl_config.h: build-1854964029
Event.dot: build-1854964029

NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/%.o: ../NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Vico/Desktop/26NUEDC/Code/main/NUEDC2026" -I"C:/Users/Vico/Desktop/26NUEDC/Code/main/NUEDC2026/Debug" -I"C:/ti/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"NUEDC/mspm0-modules-main/mspm0-modules-main/Examples/bno08x-uart-rvc-oled-hardware-spi/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"


