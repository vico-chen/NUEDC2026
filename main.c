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
#include "motor_control.h"
#include "car_control.h"
#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "oled.h"
#include "task_manager.h"
#include "task_executor.h"
#include "stopwatch.h"

#include <stdbool.h>
#include <stdint.h>

/* ======================== 全局运行参数 ======================== */
#define MOTOR_MAX_TARGET_RPM (1000U)
#define UART_COMMAND_BUFFER_SIZE (128U)
#define OPENMV_DEBUG_RX_BUFFER_SIZE (64U)
#define UART3_FORWARD_RX_BUFFER_SIZE (128U)
#define UART3_FORWARD_BYTES_PER_SERVICE (16U)
#define CAR_DEFAULT_SPEED_RPM (200)
#define CAR_DEFAULT_TURN_INNER_PERCENT (50U)
#define GRAYSCALE_STREAM_PERIOD_SAMPLES (10U) /* 10 × 10 ms = 100 ms */

/*
 * Wheel layout:
 *   C left-front, B right-front
 *   D left-rear,  A right-rear
 * Right-side motors are typically mirrored; flip one sign if a wheel
 * runs opposite to the chassis command.
 */
#define MOTOR_A_FORWARD_SIGN (-1)
#define MOTOR_B_FORWARD_SIGN (-1)
#define MOTOR_C_FORWARD_SIGN (1)
#define MOTOR_D_FORWARD_SIGN (1)

/* ======================== 四路电机硬件配置 ======================== */
static MotorControl gMotorA;
static const MotorControl_Config gMotorAConfig = {
    .pwmInstance = PWM_MOTOR_AB_INST,
    .pwmChannel = GPIO_PWM_MOTOR_AB_C0_IDX,
    .directionIn1Port = GPIO_MOTOR_A_AIN_1_PORT,
    .directionIn2Port = GPIO_MOTOR_A_AIN_2_PORT,
    .directionIn1Pin = GPIO_MOTOR_A_AIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_A_AIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_A_EA_1_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_A_EB_1_PORT,
    .encoderPhaseAPin = GPIO_MOTOR_A_EA_1_PIN,
    .encoderPhaseBPin = GPIO_MOTOR_A_EB_1_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .zeroSpeedBrakeMaxPercent = 40.0f,
};

static MotorControl gMotorB;
static const MotorControl_Config gMotorBConfig = {
    .pwmInstance = PWM_MOTOR_AB_INST,
    .pwmChannel = GPIO_PWM_MOTOR_AB_C1_IDX,
    .directionIn1Port = GPIO_MOTOR_B_BIN_1_PORT,
    .directionIn2Port = GPIO_MOTOR_B_BIN_2_PORT,
    .directionIn1Pin = GPIO_MOTOR_B_BIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_B_BIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_B_EA_2_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_B_EB_2_PORT,
    .encoderPhaseAPin = GPIO_MOTOR_B_EA_2_PIN,
    .encoderPhaseBPin = GPIO_MOTOR_B_EB_2_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .zeroSpeedBrakeMaxPercent = 40.0f,
};

static MotorControl gMotorC;
static const MotorControl_Config gMotorCConfig = {
    .pwmInstance = PWM_MOTOR_CD_INST,
    .pwmChannel = GPIO_PWM_MOTOR_CD_C0_IDX,
    .directionIn1Port = GPIO_MOTOR_C_PORT,
    .directionIn2Port = GPIO_MOTOR_C_PORT,
    .directionIn1Pin = GPIO_MOTOR_C_CIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_C_CIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_C_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_C_PORT,
    .encoderPhaseAPin = GPIO_MOTOR_C_EA_3_PIN,
    .encoderPhaseBPin = GPIO_MOTOR_C_EB_3_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .zeroSpeedBrakeMaxPercent = 40.0f,
};

static MotorControl gMotorD;
static const MotorControl_Config gMotorDConfig = {
    .pwmInstance = PWM_MOTOR_CD_INST,
    .pwmChannel = GPIO_PWM_MOTOR_CD_C1_IDX,
    .directionIn1Port = GPIO_MOTOR_D_DIN_1_PORT,
    .directionIn2Port = GPIO_MOTOR_D_DIN_2_PORT,
    .directionIn1Pin = GPIO_MOTOR_D_DIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_D_DIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_D_EA_4_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_D_EB_4_PORT,
    .encoderPhaseAPin = GPIO_MOTOR_D_EA_4_PIN,
    .encoderPhaseBPin = GPIO_MOTOR_D_EB_4_PIN,
    .pwmPeriodCounts = 100U,
    .encoderPpr = 13U,
    .gearRatio = 20U,
    .encoderDecodeMultiplier = 2U,
    .sampleRateHz = 100U,
    .reportSamples = 10U,
    .maxTargetRpm = (int16_t) MOTOR_MAX_TARGET_RPM,
    .zeroSpeedDeadbandCounts = 1,
    .kp = 5.0f,
    .ki = 1.0f,
    .kd = 0.2f,
    .outputMaxPercent = 99.0f,
    .zeroSpeedBrakeMaxPercent = 40.0f,
};

/* ======================== 车辆运动配置 ======================== */
static CarControl gCar;
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

/* ======================== 串口接收与外设状态 ======================== */
static volatile char gUartCommand[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t gUartCommandLength;
static volatile bool gUartCommandReady;
static volatile bool gControlSampleDue;

/* OpenMV 串口接收监视，由 UART0 的 O/O1/O0 命令控制。 */
static volatile uint8_t
    gOpenMvDebugRxBuffer[OPENMV_DEBUG_RX_BUFFER_SIZE];
static volatile uint8_t gOpenMvDebugRxHead;
static volatile uint8_t gOpenMvDebugRxTail;
static volatile bool gOpenMvDebugRxOverflow;
static volatile bool gOpenMvDebugEnabled;

/*
 * UART3 接收转发队列：
 * 中断中只保存数据，主循环再原样发到 UART0，避免在中断中阻塞。
 * 缓冲区大小必须是 2 的整数次幂，便于通过位与完成环形回绕。
 */
static volatile uint8_t
    gUart3ForwardRxBuffer[UART3_FORWARD_RX_BUFFER_SIZE];
static volatile uint8_t gUart3ForwardRxHead;
static volatile uint8_t gUart3ForwardRxTail;
static volatile bool gUart3ForwardRxOverflow;

static bool gOledReady;
static bool gGrayscaleStreamEnabled;
static uint8_t gGrayscaleStreamDivider;

/* ======================== 巡线与任务状态 ======================== */
static LineTracking gLineTracking;
static TaskExecutor gTaskExecutor;
static bool gMotorStatusStreamEnabled;
static bool gMotorStatusReportOnce;

/* 全局毫秒时基，同时提供给 stopwatch.c 读取。 */
volatile uint64_t systick_ms = 0;

/* 通过调试串口 UART0 阻塞发送一个以 '\0' 结尾的字符串。 */
static void UART_sendString(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) *text);
        text++;
    }
}

/* 把 UART0 命令中 U3 后面的正文发送到 UART3，并自动补上 CRLF。 */
static void UART3_sendCommandLine(uint8_t startIndex)
{
    while (gUartCommand[startIndex] != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_1_INST, (uint8_t) gUartCommand[startIndex]);
        startIndex++;
    }
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t) '\r');
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t) '\n');
}

/* 不依赖 printf，将有符号 32 位整数转换为十进制文本发送。 */
static void UART_sendInt32(int32_t value)
{
    char digits[10];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (value < 0) {
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

/* 发送一个固定两位的十六进制字节。 */
static void UART_sendHex8(uint8_t value)
{
    static const char hexDigits[] = "0123456789ABCDEF";

    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[(value >> 4) & 0x0FU]);
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[value & 0x0FU]);
}

/* 输出 UART0 可用的调试与控制命令。 */
static void UART_printHelp(void)
{
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
    UART_sendString("[Status]\r\n");
    UART_sendString("  G / G1 / G0 grayscale once/VOFA stream/off\r\n");
    UART_sendString("  M / M1 / M0 motor once/VOFA FireWater stream/off\r\n");
    UART_sendString("  O / O1 / O0 OpenMV RX status/on/off\r\n");
    UART_sendString("  U3 text     send text + CRLF to UART3 (9600 8N1)\r\n");
    UART_sendString("              UART3 RX is forwarded to UART0\r\n");
    UART_sendString("  H or ?      this help\r\n");
    UART_sendString("=====================================\r\n");
}

/* 上电后打印固件版本、硬件配置和当前功能提示。 */
static void UART_printBanner(void)
{
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
    UART_sendString(">>> Send H or ? for full command list\r\n");
}

static void UART_hintHelp(void)
{
    UART_sendString("  (Send H for help)\r\n");
}

static void UART_reportMotorStatus(
    char motorName, const MotorControl_Status *status)
{
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

/*
 * VOFA+ FireWater 电机数据帧。
 *
 * 通道顺序固定为：
 *   A目标/A速度/A计数/A PWM，
 *   B目标/B速度/B计数/B PWM，
 *   C目标/C速度/C计数/C PWM，
 *   D目标/D速度/D计数/D PWM。
 *
 * speedRpmTimes10 直接按一位小数发送，避免使用 printf 和浮点格式化，
 * 减少单片机的代码体积及串口发送开销。
 */
static void UART_sendVofaMotorChannels(const MotorControl_Status *status)
{
    int32_t speedTimes10 = status->speedRpmTimes10;
    uint32_t speedMagnitude;

    UART_sendInt32((int32_t) status->targetRpm);
    UART_sendString(",");

    if (speedTimes10 < 0) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '-');
        speedMagnitude =
            (uint32_t) (-(speedTimes10 + 1)) + 1U;
    } else {
        speedMagnitude = (uint32_t) speedTimes10;
    }
    UART_sendInt32((int32_t) (speedMagnitude / 10U));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '.');
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) ('0' + (speedMagnitude % 10U)));

    UART_sendString(",");
    UART_sendInt32(status->encoderCounts);
    UART_sendString(",");
    UART_sendInt32((int32_t) status->pwmPercent);
}

static void UART_reportMotorStatusVofa(
    const MotorControl_Status *motorA,
    const MotorControl_Status *motorB,
    const MotorControl_Status *motorC,
    const MotorControl_Status *motorD)
{
    /* FireWater 必须使用换行符结束一帧。 */
    UART_sendString("motor:");
    UART_sendVofaMotorChannels(motorA);
    UART_sendString(",");
    UART_sendVofaMotorChannels(motorB);
    UART_sendString(",");
    UART_sendVofaMotorChannels(motorC);
    UART_sendString(",");
    UART_sendVofaMotorChannels(motorD);
    UART_sendString("\r\n");
}

/* 按命令请求，以可读文本或 VOFA 格式输出四路电机状态。 */
static void UART_serviceMotorStatus(void)
{
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
        if (gMotorStatusStreamEnabled) {
            UART_reportMotorStatusVofa(&motorAStatus, &motorBStatus,
                &motorCStatus, &motorDStatus);
        } else {
            /* 单次查询仍使用带字段名的可读格式。 */
            UART_reportMotorStatus('A', &motorAStatus);
            UART_sendString(",");
            UART_reportMotorStatus('B', &motorBStatus);
            UART_sendString(",");
            UART_reportMotorStatus('C', &motorCStatus);
            UART_sendString(",");
            UART_reportMotorStatus('D', &motorDStatus);
            UART_sendString("\r\n");
        }
        motorAStatusReady = false;
        motorBStatusReady = false;
        motorCStatusReady = false;
        motorDStatusReady = false;
        if (gMotorStatusReportOnce) {
            gMotorStatusReportOnce = false;
        }
    }
}
/* 读取并输出一次八路灰度传感器的数字状态。 */
static void UART_reportGrayscale(void)
{
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

static void UART_reportGrayscaleVofa(const uint8_t *values)
{
    uint8_t i;

    UART_sendString("gray:");
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (i > 0U) {
            UART_sendString(",");
        }
        UART_sendInt32((int32_t) values[i]);
    }
    UART_sendString("\r\n");
}

/* 清除巡线 PID、传感器滤波以及丢线方向记忆。 */
static void OpenMvDebug_service(void)
{
    uint8_t serviced = 0U;

    if (!gOpenMvDebugEnabled) {
        return;
    }

    if (gOpenMvDebugRxOverflow) {
        gOpenMvDebugRxOverflow = false;
        UART_sendString("OPENMV_RX BUFFER OVERFLOW\r\n");
    }

    while ((gOpenMvDebugRxTail != gOpenMvDebugRxHead) &&
           (serviced < 1U)) {
        uint8_t rxData = gOpenMvDebugRxBuffer[gOpenMvDebugRxTail];

        gOpenMvDebugRxTail = (uint8_t)
            ((gOpenMvDebugRxTail + 1U) &
            (OPENMV_DEBUG_RX_BUFFER_SIZE - 1U));

        UART_sendString("OPENMV_RX ascii=");
        if ((rxData >= 0x20U) && (rxData <= 0x7EU)) {
            DL_UART_Main_transmitDataBlocking(UART_0_INST, rxData);
        } else {
            UART_sendString(".");
        }
        UART_sendString(" hex=0x");
        UART_sendHex8(rxData);
        UART_sendString("\r\n");
        serviced++;
    }
}

/*
 * 每轮最多转发一小批数据。UART0 比 UART3 快，因此既能及时清空队列，
 * 又不会让连续串口数据长时间占用主循环。
 */
static void UART3Forward_service(void)
{
    uint8_t serviced = 0U;

    if (gUart3ForwardRxOverflow) {
        gUart3ForwardRxOverflow = false;
        UART_sendString("\r\nUART3_RX_BUFFER_OVERFLOW\r\n");
    }

    while ((gUart3ForwardRxTail != gUart3ForwardRxHead) &&
           (serviced < UART3_FORWARD_BYTES_PER_SERVICE)) {
        uint8_t rxData = gUart3ForwardRxBuffer[gUart3ForwardRxTail];

        gUart3ForwardRxTail = (uint8_t)
            ((gUart3ForwardRxTail + 1U) &
             (UART3_FORWARD_RX_BUFFER_SIZE - 1U));
        DL_UART_Main_transmitDataBlocking(UART_0_INST, rxData);
        serviced++;
    }
}

static bool Car_parseUnsignedValue(
    uint8_t *index, uint16_t maximum, uint16_t *value)
{
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
 * Parse optional signed RPM after a motor letter.
 * Empty => 0. Accepts forms like "200", "-150", "+80".
 */
/* 解析可选的带符号 RPM；参数缺省时沿用上一速度。 */
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

/* 单电机测试时补偿该电机的车辆安装方向符号。 */
static void Car_setSingleMotorChassisRpm(
    char motorName, int16_t chassisRpm)
{
    MotorControl *motor;
    int8_t forwardSign;
    int16_t motorRpm;

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

/* 解析普通车辆动作命令的速度和内侧轮比例。 */
static bool Car_parseCommandParameters(uint8_t startIndex,
    int16_t *speedRpm, bool *speedSpecified,
    uint8_t *turnInnerPercent, bool *turnSpecified)
{
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

/* 取出一条完整 UART0 命令并分派到电机、车辆或调试功能。 */
static void Car_processUartCommand(void)
{
    uint8_t index = 0U;
    char command;
    char turnCommand = '\0';
    CarControl_Motion motion;
    int16_t speedRpm;
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
    if ((command >= 'a') && (command <= 'z')) {
        command = (char) (command - ('a' - 'A'));
    }
    index++;

    if ((command == 'H') || (command == '?')) {
        UART_printHelp();
        return;
    }

    /*
     * U3 <正文>：把正文发送到 UART3，并自动追加 \r\n。
     * 正文保持原来的大小写；只输入 U3 时发送一组空的 CRLF。
     */
    if (command == 'U') {
        if (gUartCommand[index] != '3') {
            UART_sendString(
                "UART3_FORMAT_ERROR usage: U3 <text>\r\n");
            UART_hintHelp();
            return;
        }
        index++;
        if ((gUartCommand[index] != '\0') &&
            (gUartCommand[index] != ' ') &&
            (gUartCommand[index] != '\t')) {
            UART_sendString(
                "UART3_FORMAT_ERROR usage: U3 <text>\r\n");
            UART_hintHelp();
            return;
        }
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }

        UART3_sendCommandLine(index);
        UART_sendString("UART3_TX_OK\r\n");
        return;
    }

    /* 手动运动命令优先级更高，执行前先终止正在运行的自动任务。 */
    if (TaskExecutor_isRunning(&gTaskExecutor) &&
        ((command == 'I') || (command == 'X') ||
         (command == 'F') || (command == 'B') ||
         (command == 'L') || (command == 'R') ||
         (command == 'W'))) {
        TaskExecutor_reset(&gTaskExecutor);
    }

    /* 关闭手动巡线，避免它和新的运动命令同时控制车辆。 */
    if (LineTracking_isEnabled(&gLineTracking) && (command != 'I') &&
        ((command == 'X') || (command == 'F') || (command == 'B') ||
         (command == 'L') || (command == 'R') ||
         (command == 'W'))) {
        LineTracking_setEnabled(&gLineTracking, false);
        LineTracking_setDebugEnabled(&gLineTracking, false);
        LineTracking_reset(&gLineTracking);
    }

    if ((command == 'F') || (command == 'B')) {
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
        /* X 为紧急停车：立即撤销所有电机驱动。 */
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

        /* Parse I form:
         *   I      : start with previous speed (debug=0)
         *   I0     : stop
         *   I1     : start debug (keep previous speed)
         *   I<rpm> : start with speed=<rpm> (debug=0)
         *
         * Important: do not decide by first digit only.
         * E.g. "I 10" must become speed=10, not debug=1.
         */
        if (gUartCommand[index] == '\0') {
            /* start with previous speed */
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
                setSpeed = false; /* keep previous speed */
            } else {
                setSpeed = true;
            }
            /* indexAfter unused beyond end-check; setSpeed/base speed below. */
            (void) indexAfter;
        }

        if (setSpeed) {
            LineTracking_setSpeed(&gLineTracking, (int16_t) newSpeed);
        }
        if (LineTracking_getSpeed(&gLineTracking) < 1) {
            LineTracking_setSpeed(
                &gLineTracking, CAR_DEFAULT_SPEED_RPM);
        }

        LineTracking_setEnabled(&gLineTracking, true);
        LineTracking_setDebugEnabled(&gLineTracking, debug);
        if (debug) {
            /* VOFA 连续流互斥，保证每一帧的通道数和含义一致。 */
            gGrayscaleStreamEnabled = false;
            gMotorStatusStreamEnabled = false;
            gMotorStatusReportOnce = false;
        }
        LineTracking_reset(&gLineTracking);
        CarControl_stop(&gCar);

        UART_sendString("LINE_STARTED speed=");
        UART_sendInt32(
            (int32_t) LineTracking_getSpeed(&gLineTracking));
        UART_sendString(debug ? " debug=1\r\n" : " debug=0\r\n");
        return;
    }
    if (command == 'O') {
        char option;

        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        option = gUartCommand[index];

        if (option == '\0') {
            UART_sendString(gOpenMvDebugEnabled ?
                "OPENMV_DEBUG_ON\r\n" :
                "OPENMV_DEBUG_OFF\r\n");
            return;
        }

        if ((option != '0') && (option != '1')) {
            UART_sendString(
                "O_FORMAT_ERROR usage: O | O1 | O0\r\n");
            UART_hintHelp();
            return;
        }
        index++;
        while ((gUartCommand[index] == ' ') ||
               (gUartCommand[index] == '\t')) {
            index++;
        }
        if (gUartCommand[index] != '\0') {
            UART_sendString(
                "O_FORMAT_ERROR usage: O | O1 | O0\r\n");
            UART_hintHelp();
            return;
        }

        if (option == '1') {
            /*
             * Reset the single-producer/single-consumer queue before
             * enabling the UART1 ISR producer.
             */
            gOpenMvDebugRxHead = 0U;
            gOpenMvDebugRxTail = 0U;
            gOpenMvDebugRxOverflow = false;
            gOpenMvDebugEnabled = true;
            UART_sendString("OPENMV_DEBUG_ON\r\n");
        } else {
            /* Stop ISR writes before clearing the queue. */
            gOpenMvDebugEnabled = false;
            gOpenMvDebugRxHead = 0U;
            gOpenMvDebugRxTail = 0U;
            gOpenMvDebugRxOverflow = false;
            UART_sendString("OPENMV_DEBUG_OFF\r\n");
        }
        return;
    }
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
                /* 开启灰度流时关闭其他 VOFA 连续流。 */
                LineTracking_setDebugEnabled(&gLineTracking, false);
                gMotorStatusStreamEnabled = false;
                gMotorStatusReportOnce = false;
                gGrayscaleStreamEnabled = true;
                gGrayscaleStreamDivider = 0U;
                UART_sendString("GRAY_STREAM_ON VOFA_FIREWATER\r\n");
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
                /* 开启电机流时关闭其他 VOFA 连续流。 */
                LineTracking_setDebugEnabled(&gLineTracking, false);
                gGrayscaleStreamEnabled = false;
                gMotorStatusStreamEnabled = true;
                gMotorStatusReportOnce = false;
                UART_sendString("MOTOR_STATUS_STREAM_ON VOFA_FIREWATER\r\n");
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

/* ======================== 主程序入口 ======================== */
int main(void)
{
    SYSCFG_DL_init();

    /*
     * UART3 的内容始终转发到调试串口 UART0。显式打开外设接收中断，
     * 这样旧的 SysConfig 生成文件也能立即工作。
     */
    DL_UART_Main_enableInterrupt(
        UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /*
     * OpenMV may send its initial digit immediately after power-up. Enable
     * UART1 RX before the OLED setup so that an early OpenMV result is not
     * lost.
     */
    NVIC_ClearPendingIRQ(UART_OPENMV_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_OPENMV_INST_INT_IRQN);

    gOledReady = OLED_Init();

    /*
     * Wheel map: A right-rear, B right-front, C left-front, D left-rear.
     * Arc command: FL/FR/BL/BR [rpm] [inner wheel percent].
     * Straight/pivot command: F/B/L/R [rpm], X stops.
     * Target 0 actively brakes to the deadband, then short-brakes.
     */
    MotorControl_init(&gMotorA, &gMotorAConfig);
    MotorControl_init(&gMotorB, &gMotorBConfig);
    MotorControl_init(&gMotorC, &gMotorCConfig);
    MotorControl_init(&gMotorD, &gMotorDConfig);
    CarControl_init(&gCar, &gCarConfig);
    Grayscale_Sensor_Init();

    {
        const LineTracking_Config lineTrackingConfig = {
            .car = &gCar,
            .sendString = UART_sendString,
            .sendInt32 = UART_sendInt32,
        };
        LineTracking_init(&gLineTracking, &lineTrackingConfig,
            CAR_DEFAULT_SPEED_RPM);
    }

    UART_printBanner();
    UART_sendString(gOledReady ? "OLED ready.\r\n"
                               : "OLED init failed: check I2C address/wiring.\r\n");
    TaskManager_init(gOledReady);
    {
        TaskExecutor_Config taskConfig = {
            .car = &gCar,
            .lineTracking = &gLineTracking,
            .motors = {&gMotorA, &gMotorB, &gMotorC, &gMotorD},
            .log = UART_sendString,
            .oledReady = gOledReady,
        };

        TaskExecutor_init(&gTaskExecutor, &taskConfig);
    }
    UART_sendString(
        "GRAYSCALE OK  AD0=PB17 AD1=PA14 AD2=PA15 OUT=PB24\r\n");
    UART_sendString("Ready. Send H for commands.\r\n");

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MOTOR_D_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MOTOR_D_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    DL_TimerA_startCounter(TIMER_PID_INST);

    while (1) {
        if (gControlSampleDue) {
            gControlSampleDue = false;

            if (gGrayscaleStreamEnabled) {
                gGrayscaleStreamDivider++;
                if (gGrayscaleStreamDivider >=
                    GRAYSCALE_STREAM_PERIOD_SAMPLES) {
                    gGrayscaleStreamDivider = 0U;
                    {
                        uint8_t grayscaleValues[GRAYSCALE_SENSOR_CHANNELS];
                        Grayscale_Sensor_ReadAll(grayscaleValues);
                        UART_reportGrayscaleVofa(grayscaleValues);
                    }
                }
            }

            TaskManager_update();
            {
                TaskManager_Task activeTask = TaskManager_getActiveTask();
                bool taskStartPressed = TaskManager_taskStartPressed();

                TaskExecutor_update(&gTaskExecutor, activeTask,
                    taskStartPressed);
            }

            if (LineTracking_isEnabled(&gLineTracking) &&
                !TaskExecutor_isRunning(&gTaskExecutor)) {
                LineTracking_update(&gLineTracking);
            }
        }

        if (gUartCommandReady) {
            Car_processUartCommand();
            gUartCommandReady = false;
        }


        OpenMvDebug_service();
        UART3Forward_service();
        UART_serviceMotorStatus();
        __WFI();
    }
}

/* GPIO 组中断：处理 A/B/C/D 四路编码器 B 相边沿。 */
void GROUP1_IRQHandler(void)
{
    uint32_t gpioBInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, GPIO_MOTOR_A_EB_1_PIN |
            GPIO_MOTOR_B_EB_2_PIN |
            GPIO_MOTOR_C_EB_3_PIN);
    uint32_t gpioAInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, GPIO_MOTOR_D_EB_4_PIN);

    if ((gpioBInterruptStatus & GPIO_MOTOR_A_EB_1_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorA);
    }
    if ((gpioBInterruptStatus & GPIO_MOTOR_B_EB_2_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorB);
    }
    if ((gpioBInterruptStatus & GPIO_MOTOR_C_EB_3_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorC);
    }
    if ((gpioAInterruptStatus & GPIO_MOTOR_D_EB_4_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorD);
    }

    DL_GPIO_clearInterruptStatus(GPIOB, gpioBInterruptStatus);
    DL_GPIO_clearInterruptStatus(GPIOA, gpioAInterruptStatus);
}

/* 10 ms 定时中断：采样编码器、更新四路速度 PID 并置位控制周期。 */
void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_PID_INST)) {
        case DL_TIMER_IIDX_ZERO:
            MotorControl_update(&gMotorA);
            MotorControl_update(&gMotorB);
            MotorControl_update(&gMotorC);
            MotorControl_update(&gMotorD);
            gControlSampleDue = true;

            systick_ms += 10;
            break;
        default:
            break;
    }
}

/* UART0 接收中断：收集一行调试命令，遇到回车后交给主循环。 */
void UART_0_INST_IRQHandler(void)
{
    uint8_t rxData;

    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            rxData = DL_UART_Main_receiveData(UART_0_INST);

            if (!gUartCommandReady) {
                if ((rxData == (uint8_t) '\r') ||
                    (rxData == (uint8_t) '\n')) {
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
                    gUartCommandLength = 0U;
                }
            }
            break;
        default:
            break;
    }
}

/* OpenMV 串口中断：解析视觉数字和左/右转向字符。 */
void UART_OPENMV_INST_IRQHandler(void)
{
    uint8_t rxData;
    uint8_t nextHead;

    if (DL_UART_Main_getPendingInterrupt(UART_OPENMV_INST) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }
    rxData = DL_UART_Main_receiveData(UART_OPENMV_INST);

    /* O1 监视模式只在中断中入队，不在中断内阻塞发送。 */
    if (gOpenMvDebugEnabled) {
        nextHead = (uint8_t) ((gOpenMvDebugRxHead + 1U) &
            (OPENMV_DEBUG_RX_BUFFER_SIZE - 1U));
        if (nextHead != gOpenMvDebugRxTail) {
            gOpenMvDebugRxBuffer[gOpenMvDebugRxHead] = rxData;
            gOpenMvDebugRxHead = nextHead;
        } else {
            gOpenMvDebugRxOverflow = true;
        }
    }

}

/* UART3 RX 中断：只入队，实际转发由主循环完成。 */
void UART_1_INST_IRQHandler(void)
{
    uint8_t rxData;
    uint8_t nextHead;

    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }

    rxData = DL_UART_Main_receiveData(UART_1_INST);
    nextHead = (uint8_t)
        ((gUart3ForwardRxHead + 1U) &
         (UART3_FORWARD_RX_BUFFER_SIZE - 1U));
    if (nextHead != gUart3ForwardRxTail) {
        gUart3ForwardRxBuffer[gUart3ForwardRxHead] = rxData;
        gUart3ForwardRxHead = nextHead;
    } else {
        gUart3ForwardRxOverflow = true;
    }
}
