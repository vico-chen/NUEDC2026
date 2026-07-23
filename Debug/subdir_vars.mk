################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../main.syscfg 

C_SRCS += \
../angle_turn_control.c \
../car_control.c \
../grayscale_sensor.c \
../main.c \
./ti_msp_dl_config.c \
C:/ti/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../motor_control.c \
../mpu6050_angle.c \
../oled_test.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./angle_turn_control.d \
./car_control.d \
./grayscale_sensor.d \
./main.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./motor_control.d \
./mpu6050_angle.d \
./oled_test.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./angle_turn_control.o \
./car_control.o \
./grayscale_sensor.o \
./main.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./motor_control.o \
./mpu6050_angle.o \
./oled_test.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"angle_turn_control.o" \
"car_control.o" \
"grayscale_sensor.o" \
"main.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"motor_control.o" \
"mpu6050_angle.o" \
"oled_test.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"angle_turn_control.d" \
"car_control.d" \
"grayscale_sensor.d" \
"main.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"motor_control.d" \
"mpu6050_angle.d" \
"oled_test.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../angle_turn_control.c" \
"../car_control.c" \
"../grayscale_sensor.c" \
"../main.c" \
"./ti_msp_dl_config.c" \
"C:/ti/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../motor_control.c" \
"../mpu6050_angle.c" \
"../oled_test.c" 

SYSCFG_SRCS__QUOTED += \
"../main.syscfg" 


