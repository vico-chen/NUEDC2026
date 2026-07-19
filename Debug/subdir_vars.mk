################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../main.syscfg 

C_SRCS += \
../car_control.c \
../main.c \
./ti_msp_dl_config.c \
C:/ti/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../motor_control.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./car_control.d \
./main.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./motor_control.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./car_control.o \
./main.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./motor_control.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"car_control.o" \
"main.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"motor_control.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"car_control.d" \
"main.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"motor_control.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../car_control.c" \
"../main.c" \
"./ti_msp_dl_config.c" \
"C:/ti/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../motor_control.c" 

SYSCFG_SRCS__QUOTED += \
"../main.syscfg" 


