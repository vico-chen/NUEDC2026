/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "board_pins.h"
#include "motor_control.h"
#include "car_control.h"
#include "mpu6050_angle.h"
#include "angle_turn_control.h"
#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "oled.h"
#include "system_time.h"
#include "stopwatch.h"
#include "task_manager.h"
#include "task_executor.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 主程序总体架构：
 *   - TIMER_PID：每 10 ms 更新四路电机速度环，并触发一次传感器任务；
 *   - GPIO GROUP1：接收四路编码器边沿并累加方向计数；
 *   - UART0：中断中只接收一行命令，具体解析和执行放在主循环；
 *   - 主循环：更新 MPU6050/定角转向/巡线，处理命令并输出状态。
 *
 * 各功能模块不直接依赖 SysConfig 生成的引脚名，硬件映射统一由
 * board_pins.h 适配，换引脚时优先只修改该文件。
 */

/* 小车通用限幅、默认速度及周期性输出参数。 */
#define MOTOR_MAX_TARGET_RPM (1000U)
#define UART_COMMAND_BUFFER_SIZE (32U)
#define CAR_DEFAULT_SPEED_RPM (200)
#define CAR_DEFAULT_TURN_INNER_PERCENT (50U)
#define ANGLE_TURN_DEFAULT_MAX_RPM (100)
#define ANGLE_TURN_LEFT_YAW_SIGN (1)
#define GRAYSCALE_STREAM_PERIOD_SAMPLES (10U) /* 10 次 × 10 ms = 100 ms */

/*
 * 从车体上方向下看的车轮布局：
 *   C 左前轮，B 右前轮
 *   D 左后轮，A 右后轮
 *
 * 左右两侧电机镜像安装，所需电机轴正方向相反。若以后更换接线后某个轮子
 * 与整车指令方向相反，只需修改对应的 FORWARD_SIGN。
 */
#define MOTOR_A_FORWARD_SIGN (-1)
#define MOTOR_B_FORWARD_SIGN (-1)
#define MOTOR_C_FORWARD_SIGN (1)
#define MOTOR_D_FORWARD_SIGN (1)

static MotorControl gMotorA;
/* A 电机：右后轮。四路电机采用相同的编码器和速度环参数。 */
static const MotorControl_Config gMotorAConfig = {
    .pwmInstance = BOARD_MOTOR_A_PWM_INSTANCE,
    .pwmChannel = BOARD_MOTOR_A_PWM_CHANNEL,
    .directionIn1Port = BOARD_MOTOR_A_DIR1_PORT,
    .directionIn2Port = BOARD_MOTOR_A_DIR2_PORT,
    .directionIn1Pin = BOARD_MOTOR_A_DIR1_PIN,
    .directionIn2Pin = BOARD_MOTOR_A_DIR2_PIN,
    .encoderPhaseAPort = BOARD_MOTOR_A_ENCODER_A_PORT,
    .encoderPhaseBPort = BOARD_MOTOR_A_ENCODER_B_PORT,
    .encoderPhaseAPin = BOARD_MOTOR_A_ENCODER_A_PIN,
    .encoderPhaseBPin = BOARD_MOTOR_A_ENCODER_B_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .speedFilterAlpha = 0.35f,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .runningBrakeMaxPercent = 8.0f,
    .stopBrakeMaxPercent = 25.0f,
};

static MotorControl gMotorB;
/* B 电机：右前轮。具体外设实例和引脚均来自 board_pins.h。 */
static const MotorControl_Config gMotorBConfig = {
    .pwmInstance = BOARD_MOTOR_B_PWM_INSTANCE,
    .pwmChannel = BOARD_MOTOR_B_PWM_CHANNEL,
    .directionIn1Port = BOARD_MOTOR_B_DIR1_PORT,
    .directionIn2Port = BOARD_MOTOR_B_DIR2_PORT,
    .directionIn1Pin = BOARD_MOTOR_B_DIR1_PIN,
    .directionIn2Pin = BOARD_MOTOR_B_DIR2_PIN,
    .encoderPhaseAPort = BOARD_MOTOR_B_ENCODER_A_PORT,
    .encoderPhaseBPort = BOARD_MOTOR_B_ENCODER_B_PORT,
    .encoderPhaseAPin = BOARD_MOTOR_B_ENCODER_A_PIN,
    .encoderPhaseBPin = BOARD_MOTOR_B_ENCODER_B_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .speedFilterAlpha = 0.35f,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .runningBrakeMaxPercent = 8.0f,
    .stopBrakeMaxPercent = 25.0f,
};

static MotorControl gMotorC;
/* C 电机：左前轮。引脚即使跨 GPIO 端口，控制层的配置格式仍保持一致。 */
static const MotorControl_Config gMotorCConfig = {
    .pwmInstance = BOARD_MOTOR_C_PWM_INSTANCE,
    .pwmChannel = BOARD_MOTOR_C_PWM_CHANNEL,
    .directionIn1Port = BOARD_MOTOR_C_DIR1_PORT,
    .directionIn2Port = BOARD_MOTOR_C_DIR2_PORT,
    .directionIn1Pin = BOARD_MOTOR_C_DIR1_PIN,
    .directionIn2Pin = BOARD_MOTOR_C_DIR2_PIN,
    .encoderPhaseAPort = BOARD_MOTOR_C_ENCODER_A_PORT,
    .encoderPhaseBPort = BOARD_MOTOR_C_ENCODER_B_PORT,
    .encoderPhaseAPin = BOARD_MOTOR_C_ENCODER_A_PIN,
    .encoderPhaseBPin = BOARD_MOTOR_C_ENCODER_B_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .speedFilterAlpha = 0.35f,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .runningBrakeMaxPercent = 8.0f,
    .stopBrakeMaxPercent = 25.0f,
};

static MotorControl gMotorD;
/* D 电机：左后轮。 */
static const MotorControl_Config gMotorDConfig = {
    .pwmInstance = BOARD_MOTOR_D_PWM_INSTANCE,
    .pwmChannel = BOARD_MOTOR_D_PWM_CHANNEL,
    .directionIn1Port = BOARD_MOTOR_D_DIR1_PORT,
    .directionIn2Port = BOARD_MOTOR_D_DIR2_PORT,
    .directionIn1Pin = BOARD_MOTOR_D_DIR1_PIN,
    .directionIn2Pin = BOARD_MOTOR_D_DIR2_PIN,
    .encoderPhaseAPort = BOARD_MOTOR_D_ENCODER_A_PORT,
    .encoderPhaseBPort = BOARD_MOTOR_D_ENCODER_B_PORT,
    .encoderPhaseAPin = BOARD_MOTOR_D_ENCODER_A_PIN,
    .encoderPhaseBPin = BOARD_MOTOR_D_ENCODER_B_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .speedFilterAlpha = 0.35f,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .runningBrakeMaxPercent = 8.0f,
    .stopBrakeMaxPercent = 25.0f,
};

static CarControl gCar;
static LineTracking gLineTracking;
static TaskExecutor gTaskExecutor;
static bool gOledReady;
/* 将四个独立电机映射为车体的前后、左右和弧线运动。 */
static const CarControl_Config gCarConfig = {
    .rightRearMotor = &gMotorA,
    .rightFrontMotor = &gMotorB,
    .leftFrontMotor = &gMotorC,
    .leftRearMotor = &gMotorD,
    .rightRearForwardSign = MOTOR_A_FORWARD_SIGN,
    .rightFrontForwardSign = MOTOR_B_FORWARD_SIGN,
    .leftFrontForwardSign = MOTOR_C_FORWARD_SIGN,
    .leftRearForwardSign = MOTOR_D_FORWARD_SIGN,
    .maximumSpeedRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .defaultSpeedRpm = CAR_DEFAULT_SPEED_RPM,
    .defaultTurnInnerPercent = CAR_DEFAULT_TURN_INNER_PERCENT,
};

static AngleTurnControl gAngleTurn;
/* 基于 MPU6050 Z 轴相对角度的定角转向状态机参数。 */
static const AngleTurnControl_Config gAngleTurnConfig = {
    .car = &gCar,
    /*
     * MPU6050 水平安装且 +Z 朝上时，左转/逆时针角度为正。
     * 若手动左转时串口报告的角度反而减小，将该符号改为 -1。
     */
    .leftTurnYawSign = ANGLE_TURN_LEFT_YAW_SIGN,
    .defaultCruiseRpm = ANGLE_TURN_DEFAULT_MAX_RPM,
    .creepRpm = 40,
    .brakeRateGain = 0.10f,
    .brakeAheadMaxDegrees = 12.0f,
    /* 旧值 2.0° 会让 90° 指令稳定停在约 88°，因此收紧到 0.4°。 */
    .angleToleranceDegrees = 0.4f,
    .stoppedRateToleranceDps = 6.0f,
    .settleSamples = 15U,
    .timeoutSamples = 1500U,
};

/*
 * 中断与主循环共享的标志必须使用 volatile：
 * UART ISR 写入命令缓冲区，定时器 ISR 置位 10 ms 采样任务。
 */
static volatile char gUartCommand[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t gUartCommandLength;
static volatile bool gUartCommandReady;
static volatile bool gMpu6050SampleDue;
static bool gMpu6050Ready;
/* 灰度传感器周期输出开关及分频计数。 */
static bool gGrayscaleStreamEnabled;
static uint8_t gGrayscaleStreamDivider;

/* 巡线 PID、输入消抖及调试输出的运行状态。 */
/* 四路电机状态支持单次报告和连续报告两种模式。 */
static bool gMotorStatusStreamEnabled;
static bool gMotorStatusReportOnce;

static void UART_sendString(const char *text)
{
    /* 调试输出采用阻塞发送，保证一条文本内部不会丢字符。 */
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) *text);
        text++;
    }
}

static void UART_sendInt32(int32_t value)
{
    /* 不依赖 printf，手工把有符号十进制整数从低位转换并逆序发送。 */
    char digits[10];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (value < 0) {
        /* 该写法也能安全处理 INT32_MIN，避免直接取负溢出。 */
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        digits[count] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
        count++;
    } while ((magnitude != 0U) && (count < sizeof(digits)));

    while (count > 0U) {
        count--;
        DL_UART_Main_transmitDataBlocking(
            UART_0_INST, (uint8_t) digits[count]);
    }
}

static void UART_sendHex8(uint8_t value)
{
    /* 一个字节固定输出两个大写十六进制字符。 */
    static const char hexDigits[] = "0123456789ABCDEF";

    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[(value >> 4) & 0x0FU]);
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[value & 0x0FU]);
}

static void UART_printHelp(void)
{
    /* 串口命令全集；命令字符串保持英文，便于终端显示和脚本解析。 */
    UART_sendString("\r\n");
    UART_sendString("======== NUEDC2026 UART HELP ========\r\n");
    UART_sendString("115200 8N1, end each cmd with Enter\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Motion]  F/B/L/R [rpm]\r\n");
    UART_sendString("  F 200     forward\r\n");
    UART_sendString("  B 200     backward\r\n");
    UART_sendString("  L 150     pivot left\r\n");
    UART_sendString("  R 150     pivot right\r\n");
    UART_sendString("  X         emergency stop\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Arc]     FL/FR/BL/BR [rpm] [inner%]\r\n");
    UART_sendString("  FL 200 70   forward+left arc\r\n");
    UART_sendString("  default rpm=200, inner%=50\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Angle]   TL/TR angle [rpm]\r\n");
    UART_sendString("  TL 90       left 90 deg (def 100rpm)\r\n");
    UART_sendString("  TR 45 80    right 45 deg @80rpm\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Single]  WA/WB/WC/WD [signed rpm]\r\n");
    UART_sendString("  wheels: C-B front, D-A rear\r\n");
    UART_sendString("  WA 200 / WB -150 / WA(=0 stop)\r\n");
    UART_sendString("  note: B alone = car BACK, use WB for motor B\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Line]    I [speed] | I0 stop | I1 debug\r\n");
    UART_sendString("  I 200     start line tracking (black line => X=1)\r\n");
    UART_sendString("  I0        stop\r\n");
    UART_sendString("  I1        start with debug prints\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Tasks]   Q1..Q6 select | QS start | Q0 cancel | Q status\r\n");
    UART_sendString("  implemented: Q2/Q5/Q6 lap, Q4 straight 2000mm\r\n");
    UART_sendString("  PA13 cycles Task2/4/5/6 | PA29 starts selected task\r\n");
    UART_sendString("  Task1/Task3 need route definitions and will not move\r\n");
    UART_sendString("\r\n");
    UART_sendString("[Status]\r\n");
    UART_sendString("  Y / Y0      yaw read / reset\r\n");
    UART_sendString("  G / G1 / G0 grayscale once/stream/off\r\n");
    UART_sendString("  M / M1 / M0 motor status once/stream/off\r\n");
    UART_sendString("  H or ?      this help\r\n");
    UART_sendString("=====================================\r\n");
}

static void UART_printBanner(void)
{
    /* 上电时只打印常用命令，完整说明通过 H 或 ? 查看。 */
    UART_sendString("\r\n");
    UART_sendString("************************************\r\n");
    UART_sendString("*  NUEDC2026 Car Controller Ready  *\r\n");
    UART_sendString("*  UART0 115200 8N1                *\r\n");
    UART_sendString("************************************\r\n");
    UART_sendString("Quick start:\r\n");
    UART_sendString("  F 200   forward | X stop\r\n");
    UART_sendString("  TL 90   turn left 90 deg\r\n");
    UART_sendString("  G       read grayscale\r\n");
    UART_sendString("  Y       read yaw angle\r\n");
    UART_sendString("  I 200   line tracking | I0 stop\r\n");
    UART_sendString("  PA13 select task | PA29 start | Q2/QS via UART\r\n");
    UART_sendString(">>> Send H or ? for full command list\r\n");
}

static void UART_hintHelp(void)
{
    /* 所有格式错误共用的简短提示。 */
    UART_sendString("  (Send H for help)\r\n");
}

static void UART_reportMotorStatus(
    char motorName, const MotorControl_Status *status)
{
    /* 输出格式固定，便于保存日志后用脚本解析速度、计数和 PWM。 */
    uint32_t rpmMagnitude;

    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) motorName);
    UART_sendString("[");
    UART_sendString("target_rpm=");
    UART_sendInt32((int32_t) status->targetRpm);
    UART_sendString(",speed_rpm=");
    if (status->speedRpmTimes10 < 0) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '-');
        rpmMagnitude = (uint32_t) (-status->speedRpmTimes10);
    } else {
        rpmMagnitude = (uint32_t) status->speedRpmTimes10;
    }
    UART_sendInt32((int32_t) (rpmMagnitude / 10U));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '.');
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) ('0' + (rpmMagnitude % 10U)));
    UART_sendString(",counts_100ms=");
    UART_sendInt32(status->encoderCounts);
    UART_sendString(",pwm_percent=");
    UART_sendInt32((int32_t) status->pwmPercent);
    UART_sendString("]");
}

static void UART_serviceMotorStatus(void)
{
    /*
     * 四路状态并非同一条语句产生，因此先分别缓存；四路都准备好后再整行输出，
     * 确保同一行 A/B/C/D 属于同一个相邻报告周期。
     */
    static MotorControl_Status motorAStatus;
    static MotorControl_Status motorBStatus;
    static MotorControl_Status motorCStatus;
    static MotorControl_Status motorDStatus;
    static bool motorAStatusReady;
    static bool motorBStatusReady;
    static bool motorCStatusReady;
    static bool motorDStatusReady;

    if (!gMotorStatusStreamEnabled && !gMotorStatusReportOnce) {
        motorAStatusReady = false;
        motorBStatusReady = false;
        motorCStatusReady = false;
        motorDStatusReady = false;
        return;
    }

    if (MotorControl_takeStatus(&gMotorA, &motorAStatus)) {
        motorAStatusReady = true;
    }
    if (MotorControl_takeStatus(&gMotorB, &motorBStatus)) {
        motorBStatusReady = true;
    }
    if (MotorControl_takeStatus(&gMotorC, &motorCStatus)) {
        motorCStatusReady = true;
    }
    if (MotorControl_takeStatus(&gMotorD, &motorDStatus)) {
        motorDStatusReady = true;
    }

    if (motorAStatusReady && motorBStatusReady && motorCStatusReady &&
        motorDStatusReady) {
        UART_reportMotorStatus('A', &motorAStatus);
        UART_sendString(",");
        UART_reportMotorStatus('B', &motorBStatus);
        UART_sendString(",");
        UART_reportMotorStatus('C', &motorCStatus);
        UART_sendString(",");
        UART_reportMotorStatus('D', &motorDStatus);
        UART_sendString("\r\n");
        motorAStatusReady = false;
        motorBStatusReady = false;
        motorCStatusReady = false;
        motorDStatusReady = false;
        if (gMotorStatusReportOnce) {
            gMotorStatusReportOnce = false;
        }
    }
}
static void UART_reportZAngle(void)
{
    /* 浮点角度放大 10 倍后取整，避免引入完整 printf 浮点库。 */
    float angle = MPU6050_Angle_getZDegrees();
    int32_t angleTimes10;
    uint32_t magnitude;

    if (!gMpu6050Ready) {
        UART_sendString("MPU6050_ERROR\r\n");
        return;
    }

    angleTimes10 =
        (int32_t) ((angle >= 0.0f) ? (angle * 10.0f + 0.5f)
                                   : (angle * 10.0f - 0.5f));
    UART_sendString("z_angle=");
    if (angleTimes10 < 0) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '-');
        magnitude = (uint32_t) (-(angleTimes10 + 1)) + 1U;
    } else {
        magnitude = (uint32_t) angleTimes10;
    }
    UART_sendInt32((int32_t) (magnitude / 10U));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '.');
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) ('0' + (magnitude % 10U)));
    UART_sendString(" deg\r\n");
}

static void UART_reportGrayscale(void)
{
    /* 按物理丝印 X1～X8 的顺序输出八路数字量。 */
    uint8_t values[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t i;

    Grayscale_Sensor_ReadAll(values);
    UART_sendString("GRAY");
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        UART_sendString(" X");
        UART_sendInt32((int32_t) (i + 1U));
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) ':');
        UART_sendInt32((int32_t) values[i]);
    }
    UART_sendString("\r\n");
}

static bool Car_parseUnsignedValue(
    uint8_t *index, uint16_t maximum, uint16_t *value)
{
    /* 从当前位置解析无符号十进制数，同时在乘 10 前检查上限溢出。 */
    uint16_t parsedValue = 0U;
    bool hasDigit = false;

    while ((gUartCommand[*index] >= '0') &&
           (gUartCommand[*index] <= '9')) {
        uint16_t digit =
            (uint16_t) (gUartCommand[*index] - '0');

        if (parsedValue > (uint16_t) ((maximum - digit) / 10U)) {
            return false;
        }
        parsedValue = (uint16_t) ((parsedValue * 10U) + digit);
        hasDigit = true;
        (*index)++;
    }

    if (!hasDigit) {
        return false;
    }
    *value = parsedValue;
    return true;
}

/*
 * 解析单电机字母后的可选有符号转速。
 * 参数为空时按 0 rpm 处理，也接受 “200”“-150”“+80” 等格式。
 */
static bool Car_parseOptionalSignedRpm(
    uint8_t startIndex, int16_t *rpm)
{
    uint8_t index = startIndex;
    bool negative = false;
    uint16_t magnitude;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (gUartCommand[index] == '\0') {
        *rpm = 0;
        return true;
    }

    if (gUartCommand[index] == '-') {
        negative = true;
        index++;
    } else if (gUartCommand[index] == '+') {
        index++;
    }

    if (!Car_parseUnsignedValue(
            &index, MOTOR_MAX_TARGET_RPM, &magnitude)) {
        return false;
    }

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (gUartCommand[index] != '\0') {
        return false;
    }

    *rpm = negative ? (int16_t) (-(int16_t) magnitude)
                    : (int16_t) magnitude;
    return true;
}

static void Car_setSingleMotorChassisRpm(
    char motorName, int16_t chassisRpm)
{
    MotorControl *motor;
    int8_t forwardSign;
    int16_t motorRpm;

    /* 根据车轮名称找到电机对象及其“车体前进”方向符号。 */
    switch (motorName) {
        case 'A':
            motor = &gMotorA;
            forwardSign = MOTOR_A_FORWARD_SIGN;
            break;
        case 'B':
            motor = &gMotorB;
            forwardSign = MOTOR_B_FORWARD_SIGN;
            break;
        case 'C':
            motor = &gMotorC;
            forwardSign = MOTOR_C_FORWARD_SIGN;
            break;
        case 'D':
            motor = &gMotorD;
            forwardSign = MOTOR_D_FORWARD_SIGN;
            break;
        default:
            return;
    }

    /* 单电机调试会打断正在运行的定角转向状态机。 */
    AngleTurnControl_cancel(&gAngleTurn);
    motorRpm = (int16_t) (chassisRpm * forwardSign);
    MotorControl_setTargetRpm(motor, motorRpm);

    UART_sendString("MOTOR_");
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) motorName);
    UART_sendString(" chassis_rpm=");
    UART_sendInt32((int32_t) chassisRpm);
    UART_sendString(" motor_rpm=");
    UART_sendInt32((int32_t) motorRpm);
    UART_sendString("\r\n");
}

static bool Car_parseAngleTurnParameters(
    uint8_t startIndex, float *angleDegrees, int16_t *maximumRpm)
{
    /* TL/TR 支持 0.1° 分辨率角度，后面的最大转速参数可以省略。 */
    uint8_t index = startIndex;
    uint16_t wholeDegrees;
    uint16_t fractionalTenths = 0U;
    uint16_t rpm;
    uint16_t angleTenths;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (!Car_parseUnsignedValue(&index, 360U, &wholeDegrees)) {
        return false;
    }

    /* 小数部分只读取一位，避免在嵌入式端引入通用浮点字符串解析。 */
    if (gUartCommand[index] == '.') {
        index++;
        if ((gUartCommand[index] < '0') ||
            (gUartCommand[index] > '9')) {
            return false;
        }
        fractionalTenths =
            (uint16_t) (gUartCommand[index] - '0');
        index++;
    }
    if ((gUartCommand[index] != '\0') &&
        (gUartCommand[index] != ' ') &&
        (gUartCommand[index] != '\t')) {
        return false;
    }

    angleTenths =
        (uint16_t) (wholeDegrees * 10U + fractionalTenths);
    if ((angleTenths == 0U) || (angleTenths > 3600U)) {
        return false;
    }

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    /* 未给出 rpm 时使用定角转向专用默认值。 */
    *maximumRpm = ANGLE_TURN_DEFAULT_MAX_RPM;
    if (gUartCommand[index] != '\0') {
        if (!Car_parseUnsignedValue(
                &index, MOTOR_MAX_TARGET_RPM, &rpm) ||
            (rpm == 0U)) {
            return false;
        }
        *maximumRpm = (int16_t) rpm;
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
    }

    if (gUartCommand[index] != '\0') {
        return false;
    }
    *angleDegrees = (float) angleTenths / 10.0f;
    return true;
}

static bool Car_parseCommandParameters(uint8_t startIndex,
    int16_t *speedRpm, bool *speedSpecified,
    uint8_t *turnInnerPercent, bool *turnSpecified)
{
    /*
     * 解析普通运动命令的两个可选参数：
     *   第一个是外侧轮速度 rpm，第二个是弧线内侧轮速度百分比。
     * 缺省参数沿用 CarControl 当前保存的设置。
     */
    uint8_t index = startIndex;
    uint16_t value;

    *speedRpm = CarControl_getSpeedRpm(&gCar);
    *turnInnerPercent = CarControl_getTurnInnerPercent(&gCar);
    *speedSpecified = false;
    *turnSpecified = false;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (gUartCommand[index] == '\0') {
        return true;
    }

    if (!Car_parseUnsignedValue(
            &index, MOTOR_MAX_TARGET_RPM, &value)) {
        return false;
    }
    *speedRpm = (int16_t) value;
    *speedSpecified = true;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (gUartCommand[index] == '\0') {
        return true;
    }

    if (!Car_parseUnsignedValue(&index, 100U, &value)) {
        return false;
    }
    *turnInnerPercent = (uint8_t) value;
    *turnSpecified = true;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    return (gUartCommand[index] == '\0');
}

static void Car_processUartCommand(void)
{
    /*
     * 命令解析采用“命令字 + 可选参数”的轻量状态机。
     * ISR 已在行尾补 '\0'，因此此处只处理完整命令，不与接收中断争用缓冲区。
     */
    uint8_t index = 0U;
    char command;
    char turnCommand = '\0';
    CarControl_Motion motion;
    int16_t speedRpm;
    int16_t angleTurnMaximumRpm;
    float angleTurnDegrees;
    uint8_t turnInnerPercent;
    bool speedSpecified;
    bool turnSpecified;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    command = gUartCommand[index];
    if (command == '\0') {
        UART_sendString("Empty cmd. Send H for help.\r\n");
        return;
    }
    /* 命令不区分大小写，统一转换为大写后再分派。 */
    if ((command >= 'a') && (command <= 'z')) {
        command = (char) (command - ('a' - 'A'));
    }
    index++;

    if ((command == 'H') || (command == '?')) {
        UART_printHelp();
        return;
    }

    /*
     * 任务命令只使用 UART0：Q1..Q6 选择、QS 启动、Q0 取消、Q 查询。
     * PA13 可循环选择已支持任务，PA29 启动键与 QS 等价。
     * 整个任务控制不需要 OpenMV、UART1 或 UART3。
     */
    if (command == 'Q') {
        char taskArgument;

        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        taskArgument = gUartCommand[index];
        if ((taskArgument >= 'a') && (taskArgument <= 'z')) {
            taskArgument =
                (char) (taskArgument - ('a' - 'A'));
        }

        if (taskArgument == '\0') {
            UART_sendString("TASK selected=");
            UART_sendInt32((int32_t) TaskManager_getSelected());
            UART_sendString(" active=");
            UART_sendInt32((int32_t)
                TaskExecutor_getActiveTask(&gTaskExecutor));
            UART_sendString(" running=");
            UART_sendInt32(TaskExecutor_isRunning(&gTaskExecutor) ?
                1 : 0);
            UART_sendString(" fault=");
            UART_sendInt32(
                TaskExecutor_lastTaskFault(&gTaskExecutor) ? 1 : 0);
            UART_sendString(" elapsed_ms=");
            UART_sendInt32((int32_t) Stopwatch_getElapsedMs());
            UART_sendString("\r\n");
            return;
        }

        index++;
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        if (gUartCommand[index] != '\0') {
            UART_sendString(
                "TASK_FORMAT_ERROR usage: Q1..Q6 | QS | Q0 | Q\r\n");
            return;
        }

        if (taskArgument == 'S') {
            if (!TaskExecutor_isRunning(&gTaskExecutor)) {
                AngleTurnControl_cancel(&gAngleTurn);
                LineTracking_setEnabled(&gLineTracking, false);
                LineTracking_reset(&gLineTracking);
            }
            (void) TaskExecutor_startSelected(&gTaskExecutor);
            return;
        }
        if (taskArgument == '0') {
            TaskExecutor_reset(&gTaskExecutor);
            CarControl_emergencyStop(&gCar);
            UART_sendString("TASK_CANCELLED\r\n");
            return;
        }
        if ((taskArgument >= '1') && (taskArgument <= '6')) {
            TaskId selected = (TaskId) (taskArgument - '0');

            if (TaskExecutor_isRunning(&gTaskExecutor)) {
                UART_sendString(
                    "TASK_BUSY (send Q0 before changing task)\r\n");
            } else {
                (void) TaskManager_select(selected);
                if (gOledReady) {
                    gOledReady = OLED_ShowTask((uint8_t) selected);
                }
                UART_sendString("TASK_SELECTED ");
                UART_sendInt32((int32_t) selected);
                if ((selected == TASK_ID_1) ||
                    (selected == TASK_ID_3)) {
                    UART_sendString(
                        " (unsupported: route not defined)\r\n");
                } else {
                    UART_sendString("\r\n");
                }
            }
            return;
        }
        UART_sendString(
            "TASK_FORMAT_ERROR usage: Q1..Q6 | QS | Q0 | Q\r\n");
        return;
    }

    /* 任何手动运动命令都会退出自动巡线，避免两个控制源同时写车轮目标值。 */
    if (TaskExecutor_isRunning(&gTaskExecutor) &&
        ((command == 'X') || (command == 'T') ||
         (command == 'F') || (command == 'B') ||
         (command == 'L') || (command == 'R') ||
         (command == 'W') || (command == 'I'))) {
        TaskExecutor_reset(&gTaskExecutor);
    }

    if (LineTracking_isEnabled(&gLineTracking) && (command != 'I') &&
        ((command == 'X') || (command == 'T') ||
         (command == 'F') || (command == 'B') ||
         (command == 'L') || (command == 'R') ||
         (command == 'W'))) {
        LineTracking_setEnabled(&gLineTracking, false);
        LineTracking_setDebugEnabled(&gLineTracking, false);
        LineTracking_reset(&gLineTracking);
    }

    /* F/B 后的 L/R 表示弧线方向，T 后的 L/R 表示定角转向方向。 */
    if ((command == 'F') || (command == 'B') || (command == 'T')) {
        turnCommand = gUartCommand[index];
        if ((turnCommand >= 'a') && (turnCommand <= 'z')) {
            turnCommand = (char) (turnCommand - ('a' - 'A'));
        }
        if ((turnCommand == 'L') || (turnCommand == 'R')) {
            index++;
        } else {
            turnCommand = '\0';
        }
    }

    if (command == 'X') {
        TaskExecutor_reset(&gTaskExecutor);
        AngleTurnControl_cancel(&gAngleTurn);
        /* X 为急停：立即撤销驱动，不等待速度 PID 缓慢减速。 */
        CarControl_emergencyStop(&gCar);
        LineTracking_setEnabled(&gLineTracking, false);
        LineTracking_setDebugEnabled(&gLineTracking, false);
        LineTracking_reset(&gLineTracking);
        UART_sendString("STOPPED\r\n");
        return;
    }
    if (command == 'I') {
        uint16_t newSpeed;
        bool setSpeed = false;
        bool debug = false;

        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }

        /* 解析 I 命令：
         *   I      ：沿用上次速度启动巡线，不输出调试信息；
         *   I0     ：停止巡线；
         *   I1     ：沿用上次速度启动巡线，并输出调试信息；
         *   I<rpm> ：以指定速度启动巡线，不输出调试信息。
         *
         * 必须解析完整数值后再判断，不能只看首位数字；
         * 例如 “I 10” 应解释为 10 rpm，而不是调试开关 I1。
         */
        if (gUartCommand[index] == '\0') {
            /* 无参数：保留上次基础速度。 */
        } else {
            uint8_t indexAfter = index;
            if (!Car_parseUnsignedValue(&indexAfter,
                    MOTOR_MAX_TARGET_RPM, &newSpeed)) {
                UART_sendString("LINE_FORMAT_ERROR usage: I [rpm] | I0 | I1\r\n");
                UART_hintHelp();
                return;
            }

            while ((gUartCommand[indexAfter] == ' ') ||
                   (gUartCommand[indexAfter] == '\t')) {
                indexAfter++;
            }
            if (gUartCommand[indexAfter] != '\0') {
                UART_sendString("LINE_FORMAT_ERROR usage: I [rpm] | I0 | I1\r\n");
                UART_hintHelp();
                return;
            }

            if (newSpeed == 0U) {
                LineTracking_setEnabled(&gLineTracking, false);
                LineTracking_setDebugEnabled(&gLineTracking, false);
                LineTracking_reset(&gLineTracking);
                CarControl_stop(&gCar);
                UART_sendString("LINE_STOPPED\r\n");
                return;
            }

            if (newSpeed == 1U) {
                debug = true;
                setSpeed = false; /* I1 只开调试，保留上次基础速度 */
            } else {
                setSpeed = true;
            }
            /* indexAfter 仅用于格式检查，实际速度在下面统一更新。 */
            (void) indexAfter;
        }

        if (setSpeed) {
            LineTracking_setSpeed(&gLineTracking, (int16_t) newSpeed);
        }
        if (LineTracking_getSpeed(&gLineTracking) < 1) {
            LineTracking_setSpeed(
                &gLineTracking, CAR_DEFAULT_SPEED_RPM);
        }

        TaskExecutor_reset(&gTaskExecutor);
        LineTracking_setEnabled(&gLineTracking, true);
        LineTracking_setDebugEnabled(&gLineTracking, debug);
        LineTracking_reset(&gLineTracking);
        AngleTurnControl_cancel(&gAngleTurn);
        CarControl_stop(&gCar);

        UART_sendString("LINE_STARTED speed=");
        UART_sendInt32((int32_t) LineTracking_getSpeed(&gLineTracking));
        UART_sendString(debug ? " debug=1\r\n" : " debug=0\r\n");
        return;
    }
    /* TL/TR：启动闭环定角转向，完成、超时或故障结果由主循环报告。 */
    if (command == 'T') {
        if ((turnCommand == '\0') ||
            !Car_parseAngleTurnParameters(
                index, &angleTurnDegrees, &angleTurnMaximumRpm)) {
            UART_sendString("ANGLE_TURN_FORMAT_ERROR\r\n");
            UART_sendString("  usage: TL|TR angle [rpm]  e.g. TL 90\r\n");
            UART_hintHelp();
            return;
        }
        if (!gMpu6050Ready) {
            UART_sendString("MPU6050_ERROR\r\n");
            return;
        }
        if (AngleTurnControl_start(&gAngleTurn,
                turnCommand == 'L', angleTurnDegrees,
                angleTurnMaximumRpm)) {
            UART_sendString("ANGLE_TURN_STARTED ");
            UART_sendString((turnCommand == 'L') ? "TL " : "TR ");
            UART_sendInt32((int32_t) (angleTurnDegrees + 0.05f));
            UART_sendString(" deg @");
            UART_sendInt32((int32_t) angleTurnMaximumRpm);
            UART_sendString(" rpm\r\n");
        }
        return;
    }
    /* Y 读取当前相对航向角；Y0 在非转向状态下清零积分角度。 */
    if (command == 'Y') {
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        if (gUartCommand[index] == '0') {
            if (AngleTurnControl_isActive(&gAngleTurn)) {
                UART_sendString("ANGLE_TURN_BUSY\r\n");
                return;
            }
            index++;
            while ((gUartCommand[index] == ' ') ||
                   (gUartCommand[index] == '\t')) {
                index++;
            }
            if (gUartCommand[index] == '\0') {
                MPU6050_Angle_reset();
                UART_sendString("z_angle reset to 0.0 deg\r\n");
            } else {
                UART_sendString("Y_FORMAT_ERROR  usage: Y | Y0\r\n");
                UART_hintHelp();
            }
            return;
        }
        if (gUartCommand[index] == '\0') {
            UART_reportZAngle();
        } else {
            UART_sendString("Y_FORMAT_ERROR  usage: Y | Y0\r\n");
            UART_hintHelp();
        }
        return;
    }
    /* G/G1/G0 分别对应灰度单次读取、周期输出和关闭周期输出。 */
    if (command == 'G') {
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        if (gUartCommand[index] == '1') {
            index++;
            while ((gUartCommand[index] == ' ') ||
                   (gUartCommand[index] == '\t')) {
                index++;
            }
            if (gUartCommand[index] == '\0') {
                gGrayscaleStreamEnabled = true;
                gGrayscaleStreamDivider = 0U;
                UART_sendString("GRAY_STREAM_ON\r\n");
            } else {
                UART_sendString("G_FORMAT_ERROR  usage: G | G1 | G0\r\n");
                UART_hintHelp();
            }
            return;
        }
        if (gUartCommand[index] == '0') {
            index++;
            while ((gUartCommand[index] == ' ') ||
                   (gUartCommand[index] == '\t')) {
                index++;
            }
            if (gUartCommand[index] == '\0') {
                gGrayscaleStreamEnabled = false;
                UART_sendString("GRAY_STREAM_OFF\r\n");
            } else {
                UART_sendString("G_FORMAT_ERROR  usage: G | G1 | G0\r\n");
                UART_hintHelp();
            }
            return;
        }
        if (gUartCommand[index] == '\0') {
            UART_reportGrayscale();
        } else {
            UART_sendString("G_FORMAT_ERROR  usage: G | G1 | G0\r\n");
            UART_hintHelp();
        }
        return;
    }
    /* M/M1/M0 分别对应电机状态单次读取、周期输出和关闭周期输出。 */
    if (command == 'M') {
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        if (gUartCommand[index] == '1') {
            index++;
            while ((gUartCommand[index] == ' ') ||
                   (gUartCommand[index] == '\t')) {
                index++;
            }
            if (gUartCommand[index] == '\0') {
                gMotorStatusStreamEnabled = true;
                gMotorStatusReportOnce = false;
                UART_sendString("MOTOR_STATUS_STREAM_ON\r\n");
            } else {
                UART_sendString("M_FORMAT_ERROR  usage: M | M1 | M0\r\n");
                UART_hintHelp();
            }
            return;
        }
        if (gUartCommand[index] == '0') {
            index++;
            while ((gUartCommand[index] == ' ') ||
                   (gUartCommand[index] == '\t')) {
                index++;
            }
            if (gUartCommand[index] == '\0') {
                gMotorStatusStreamEnabled = false;
                gMotorStatusReportOnce = false;
                UART_sendString("MOTOR_STATUS_STREAM_OFF\r\n");
            } else {
                UART_sendString("M_FORMAT_ERROR  usage: M | M1 | M0\r\n");
                UART_hintHelp();
            }
            return;
        }
        if (gUartCommand[index] == '\0') {
            gMotorStatusReportOnce = true;
            UART_sendString("MOTOR_STATUS_WAIT\r\n");
        } else {
            UART_sendString("M_FORMAT_ERROR  usage: M | M1 | M0\r\n");
            UART_hintHelp();
        }
        return;
    }
    /* WA/WB/WC/WD 用于脱离整车运动分配，单独调试某个电机。 */
    if (command == 'W') {
        char motorName;
        int16_t singleMotorRpm;

        motorName = gUartCommand[index];
        if ((motorName >= 'a') && (motorName <= 'z')) {
            motorName = (char) (motorName - ('a' - 'A'));
        }
        if ((motorName != 'A') && (motorName != 'B') &&
            (motorName != 'C') && (motorName != 'D')) {
            UART_sendString("MOTOR_FORMAT_ERROR\r\n");
            UART_sendString(
                "  usage: WA|WB|WC|WD [signed rpm]  e.g. WA 200\r\n");
            UART_sendString(
                "  note: B = car BACKWARD; WB = motor B only\r\n");
            UART_hintHelp();
            return;
        }
        index++;

        if (!Car_parseOptionalSignedRpm(index, &singleMotorRpm)) {
            UART_sendString("MOTOR_FORMAT_ERROR\r\n");
            UART_sendString(
                "  usage: WA|WB|WC|WD [signed rpm]  e.g. WA 200\r\n");
            UART_hintHelp();
            return;
        }
        Car_setSingleMotorChassisRpm(motorName, singleMotorRpm);
        return;
    }
    if ((command != 'F') && (command != 'B') &&
        (command != 'L') && (command != 'R')) {
        UART_sendString("UNKNOWN_CMD\r\n");
        UART_hintHelp();
        return;
    }
    if (!Car_parseCommandParameters(index, &speedRpm, &speedSpecified,
            &turnInnerPercent, &turnSpecified)) {
        UART_sendString("MOTION_FORMAT_ERROR\r\n");
        UART_sendString(
            "  usage: F/B/L/R [rpm] | FL/FR/BL/BR [rpm] [inner%]\r\n");
        UART_hintHelp();
        return;
    }
    if (turnSpecified && (turnCommand == '\0')) {
        UART_sendString("MOTION_FORMAT_ERROR\r\n");
        UART_sendString(
            "  inner% only for FL/FR/BL/BR\r\n");
        UART_hintHelp();
        return;
    }

    /* 将普通运动命令转换为 CarControl 的统一运动枚举。 */
    switch (command) {
        case 'F':
            if (turnCommand == 'L') {
                motion = CAR_CONTROL_FORWARD_LEFT;
            } else if (turnCommand == 'R') {
                motion = CAR_CONTROL_FORWARD_RIGHT;
            } else {
                motion = CAR_CONTROL_FORWARD;
            }
            break;
        case 'B':
            if (turnCommand == 'L') {
                motion = CAR_CONTROL_BACKWARD_LEFT;
            } else if (turnCommand == 'R') {
                motion = CAR_CONTROL_BACKWARD_RIGHT;
            } else {
                motion = CAR_CONTROL_BACKWARD;
            }
            break;
        case 'L':
            motion = CAR_CONTROL_PIVOT_LEFT;
            break;
        case 'R':
            motion = CAR_CONTROL_PIVOT_RIGHT;
            break;
        default:
            UART_sendString("UNKNOWN_CMD\r\n");
            UART_hintHelp();
            return;
    }

    /* 新的手动运动指令接管四轮目标值，先取消可能仍活动的定角控制。 */
    AngleTurnControl_cancel(&gAngleTurn);
    CarControl_setMotion(&gCar, motion, speedRpm, turnInnerPercent);

    UART_sendString("OK ");
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) command);
    if (turnCommand != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_0_INST, (uint8_t) turnCommand);
    }
    UART_sendString(" rpm=");
    UART_sendInt32((int32_t) speedRpm);
    if ((motion == CAR_CONTROL_FORWARD_LEFT) ||
        (motion == CAR_CONTROL_FORWARD_RIGHT) ||
        (motion == CAR_CONTROL_BACKWARD_LEFT) ||
        (motion == CAR_CONTROL_BACKWARD_RIGHT)) {
        UART_sendString(" inner%=");
        UART_sendInt32((int32_t) turnInnerPercent);
    }
    UART_sendString("\r\n");
}

int main(void)
{
    /* 先执行 SysConfig 生成的时钟、GPIO、定时器、UART 和 I2C 初始化。 */
    SYSCFG_DL_init();

    /* OLED 失败不会阻止车辆控制；后续显示接口共享该在线标志。 */
    gOledReady = OLED_Init();

    /*
     * 依赖顺序：先初始化四个底层电机，再初始化整车运动和定角转向模块。
     * 车轮映射为 A 右后、B 右前、C 左前、D 左后。
     * 电机目标为 0 时先主动反向制动到速度死区，随后进入短刹状态。
     */
    MotorControl_init(&gMotorA, &gMotorAConfig);
    MotorControl_init(&gMotorB, &gMotorBConfig);
    MotorControl_init(&gMotorC, &gMotorCConfig);
    MotorControl_init(&gMotorD, &gMotorDConfig);
    CarControl_init(&gCar, &gCarConfig);
    AngleTurnControl_init(&gAngleTurn, &gAngleTurnConfig);
    Grayscale_Sensor_Init();

    /* 巡线默认关闭，但预装一个可直接使用的基础速度。 */
    {
        const LineTracking_Config lineConfig = {
            .car = &gCar,
            .sendString = UART_sendString,
            .sendInt32 = UART_sendInt32,
        };
        const TaskExecutor_Config taskConfig = {
            .car = &gCar,
            .lineTracking = &gLineTracking,
            .motors = {&gMotorA, &gMotorB, &gMotorC, &gMotorD},
            .log = UART_sendString,
            .logInt32 = UART_sendInt32,
            .oledReady = &gOledReady,
        };

        LineTracking_init(&gLineTracking, &lineConfig,
            CAR_DEFAULT_SPEED_RPM);
        TaskManager_init();
        if (gOledReady) {
            gOledReady = OLED_ShowTask(
                (uint8_t) TaskManager_getSelected());
        }
        TaskExecutor_init(&gTaskExecutor, &taskConfig);
    }

    /* 输出各外设自检结果；MPU6050 标定期间必须保持车体静止约 5 秒。 */
    UART_printBanner();
    UART_sendString(gOledReady ? "OLED: task display ready.\r\n"
                              : "OLED init failed: check I2C address/wiring.\r\n");
    UART_sendString(
        "MPU6050: keep car still for 5 seconds (calibrating)...\r\n");
    gMpu6050Ready = MPU6050_Angle_init();
    if (gMpu6050Ready) {
        UART_sendString("MPU6050 ready, WHO_AM_I=0x");
        UART_sendHex8((uint8_t) MPU6050_Angle_getDeviceId());
        UART_sendString("  (Y read, Y0 reset)\r\n");
    } else {
        UART_sendString("MPU6050 init failed.\r\n");
    }
    UART_sendString(
        "GRAYSCALE OK  AD0=PB27 AD1=PB26 AD2=PB23 OUT=PA12\r\n");
    UART_sendString("Ready. Send H for commands.\r\n");

    /*
     * 清除上电阶段可能遗留的挂起标志后，再使能 UART、两组编码器 GPIO
     * 以及 10 ms PID 定时器中断。
     */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(BOARD_ENCODER_GPIOB_IRQN);
    NVIC_EnableIRQ(BOARD_ENCODER_GPIOB_IRQN);
    NVIC_ClearPendingIRQ(BOARD_ENCODER_GPIOA_IRQN);
    NVIC_EnableIRQ(BOARD_ENCODER_GPIOA_IRQN);
    NVIC_ClearPendingIRQ(TIMER_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    /* 所有软件状态就绪后最后启动周期定时器。 */
    DL_TimerA_startCounter(TIMER_PID_INST);

    while (1) {
        /*
         * 10 ms 任务由定时器 ISR 置位，在主循环执行耗时的 I2C 和串口工作，
         * 使电机 PID 中断保持短小、确定。
         */
        if (gMpu6050SampleDue) {
            AngleTurnControl_Result angleTurnResult =
                ANGLE_TURN_RESULT_NONE;

            gMpu6050SampleDue = false;
            /* 先更新角速度/积分角度，再推进定角转向状态机。 */
            if (gMpu6050Ready) {
                if (MPU6050_Angle_update()) {
                    angleTurnResult =
                        AngleTurnControl_update(&gAngleTurn);
                } else {
                    AngleTurnControl_cancel(&gAngleTurn);
                    gMpu6050Ready = false;
                    UART_sendString("MPU6050 read failed.\r\n");
                }
            }

            /* 状态机只返回一次终态结果，在这里集中转换成串口消息。 */
            if (angleTurnResult == ANGLE_TURN_RESULT_COMPLETED) {
                UART_sendString("ANGLE_TURN_DONE ");
                UART_reportZAngle();
            } else if (angleTurnResult == ANGLE_TURN_RESULT_TIMEOUT) {
                UART_sendString("ANGLE_TURN_TIMEOUT ");
                UART_reportZAngle();
            } else if (angleTurnResult == ANGLE_TURN_RESULT_FAULT) {
                UART_sendString(
                    "ANGLE_TURN_FAULT check ANGLE_TURN_LEFT_YAW_SIGN\r\n");
            }

            /* 灰度连续报告按 10:1 分频，即每 100 ms 输出一次。 */
            if (gGrayscaleStreamEnabled) {
                gGrayscaleStreamDivider++;
                if (gGrayscaleStreamDivider >=
                    GRAYSCALE_STREAM_PERIOD_SAMPLES) {
                    gGrayscaleStreamDivider = 0U;
                    UART_reportGrayscale();
                }
            }

            /* 巡线与 MPU6050 共用稳定的 10 ms 软件任务节拍。 */
            TaskManager_update10ms();
            if (TaskManager_takeChangePressed()) {
                if (TaskExecutor_isRunning(&gTaskExecutor)) {
                    UART_sendString(
                        "TASK_BUSY (PA13 change ignored while running)\r\n");
                } else {
                    TaskId selected =
                        TaskManager_selectNextSupported();

                    if (gOledReady) {
                        gOledReady =
                            OLED_ShowTask((uint8_t) selected);
                    }
                    UART_sendString("TASK_SELECTED ");
                    UART_sendInt32((int32_t) selected);
                    UART_sendString(" (PA13)\r\n");
                }
            }
            if (TaskManager_takeStartPressed()) {
                if (!TaskExecutor_isRunning(&gTaskExecutor)) {
                    AngleTurnControl_cancel(&gAngleTurn);
                    LineTracking_setEnabled(&gLineTracking, false);
                    LineTracking_reset(&gLineTracking);
                }
                (void) TaskExecutor_startSelected(&gTaskExecutor);
            }
            TaskExecutor_update10ms(&gTaskExecutor);

            if (LineTracking_isEnabled(&gLineTracking) &&
                !TaskExecutor_isRunning(&gTaskExecutor)) {
                LineTracking_update(&gLineTracking);
            }
        }

        /* 命令解析放在主循环；完成后才允许 ISR 接收下一条完整命令。 */
        if (gUartCommandReady) {
            Car_processUartCommand();
            gUartCommandReady = false;
        }

        /* 汇总四路速度状态后输出；空闲时 WFI 降低无效 CPU 占用。 */
        UART_serviceMotorStatus();
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    /*
     * GPIOA/GPIOB 的编码器 B 相中断共享 GROUP1 入口。
     * 先读取两端口的有效状态，再分别交给对应电机判断方向并累计计数。
     */
    uint32_t gpioBInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        BOARD_ENCODER_GPIOB_PORT, BOARD_ENCODER_GPIOB_MASK);
    uint32_t gpioAInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        BOARD_ENCODER_GPIOA_PORT, BOARD_ENCODER_GPIOA_MASK);

    if ((gpioBInterruptStatus & BOARD_MOTOR_A_ENCODER_B_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorA);
    }
    if ((gpioAInterruptStatus & BOARD_MOTOR_B_ENCODER_B_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorB);
    }
    if ((gpioBInterruptStatus & BOARD_MOTOR_C_ENCODER_B_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorC);
    }
    if ((gpioBInterruptStatus & BOARD_MOTOR_D_ENCODER_B_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorD);
    }

    /* 使用进入中断时读取的位掩码一次性清除已处理标志。 */
    DL_GPIO_clearInterruptStatus(
        BOARD_ENCODER_GPIOB_PORT, gpioBInterruptStatus);
    DL_GPIO_clearInterruptStatus(
        BOARD_ENCODER_GPIOA_PORT, gpioAInterruptStatus);
}

void TIMER_PID_INST_IRQHandler(void)
{
    /* 10 ms 实时节拍：四路速度环必须在中断内按固定周期更新。 */
    switch (DL_TimerA_getPendingInterrupt(TIMER_PID_INST)) {
        case DL_TIMER_IIDX_ZERO:
            SystemTime_tick10ms();
            MotorControl_update(&gMotorA);
            MotorControl_update(&gMotorB);
            MotorControl_update(&gMotorC);
            MotorControl_update(&gMotorD);
            /* 传感器、转向和巡线任务较慢，只通知主循环执行。 */
            gMpu6050SampleDue = true;
            break;
        default:
            break;
    }
}

void UART_0_INST_IRQHandler(void)
{
    uint8_t rxData;

    /*
     * ISR 只完成按行接收：回车/换行封包，主循环负责解析。
     * 当上一条命令尚未处理时暂停写缓冲区，避免覆盖正在解析的数据。
     */
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            rxData = DL_UART_Main_receiveData(UART_0_INST);

            if (!gUartCommandReady) {
                if ((rxData == (uint8_t) '\r') ||
                    (rxData == (uint8_t) '\n')) {
                    /* 空行忽略；非空命令补字符串结束符后交给主循环。 */
                    if (gUartCommandLength > 0U) {
                        gUartCommand[gUartCommandLength] = '\0';
                        gUartCommandReady = true;
                        gUartCommandLength = 0U;
                    }
                } else if (gUartCommandLength <
                           (UART_COMMAND_BUFFER_SIZE - 1U)) {
                    gUartCommand[gUartCommandLength] = (char) rxData;
                    gUartCommandLength++;
                } else {
                    /* 超长命令直接丢弃当前内容，从缓冲区开头重新接收。 */
                    gUartCommandLength = 0U;
                }
            }
            break;
        default:
            break;
    }
}
