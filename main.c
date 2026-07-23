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
#include "mpu6050_angle.h"
#include "angle_turn_control.h"
#include "grayscale_sensor.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_TARGET_RPM (1000U)
#define UART_COMMAND_BUFFER_SIZE (32U)
#define CAR_DEFAULT_SPEED_RPM (200)
#define CAR_DEFAULT_TURN_INNER_PERCENT (50U)
#define ANGLE_TURN_DEFAULT_MAX_RPM (100)
#define ANGLE_TURN_LEFT_YAW_SIGN (1)
#define GRAYSCALE_STREAM_PERIOD_SAMPLES (10U) /* 10 × 10 ms = 100 ms */

/* Line tracking config (8-ch digital grayscale, black line => output 1). */
#define LINE_TRACKING_ACTIVE_LEVEL (1U)
#define LINE_PID_KP (4.0f)
#define LINE_PID_KI (0.01f)
#define LINE_PID_KD (0.0f)
#define LINE_PID_INTEGRAL_LIMIT (2000.0f)
#define LINE_PID_DEADBAND_RPM_OFFSET (8) /* within this => straight */
#define LINE_ERR_DEADBAND (5) /* |err|<=5 treated as centered */
#define LINE_ERR_ABS_FALLBACK (30) /* when no sensor active */
#define LINE_DEBUG_PRINT_PERIOD_SAMPLES (10U) /* 10 × 10 ms = 100 ms */
/* Debounce: require this many equal samples before accepting a bit flip. */
#define LINE_SENSOR_DEBOUNCE_SAMPLES (2U)

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

static MotorControl gMotorA;
static const MotorControl_Config gMotorAConfig = {
    .pwmInstance = PWM_MOTOR_AB_INST,
    .pwmChannel = DL_TIMER_CC_0_INDEX,
    .directionIn1Port = GPIO_MOTOR_A_PORT,
    .directionIn2Port = GPIO_MOTOR_A_PORT,
    .directionIn1Pin = GPIO_MOTOR_A_AIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_A_AIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_A_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_A_PORT,
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
    .pwmChannel = DL_TIMER_CC_1_INDEX,
    .directionIn1Port = GPIO_MOTOR_B_PORT,
    .directionIn2Port = GPIO_MOTOR_B_PORT,
    .directionIn1Pin = GPIO_MOTOR_B_BIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_B_BIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_B_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_B_PORT,
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
    .pwmChannel = DL_TIMER_CC_0_INDEX,
    .directionIn1Port = GPIO_MOTOR_C_CIN_1_PORT,
    .directionIn2Port = GPIO_MOTOR_C_CIN_2_PORT,
    .directionIn1Pin = GPIO_MOTOR_C_CIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_C_CIN_2_PIN,
    .encoderPhaseAPort = GPIO_MOTOR_C_EA_3_PORT,
    .encoderPhaseBPort = GPIO_MOTOR_C_EB_3_PORT,
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
    .pwmChannel = DL_TIMER_CC_1_INDEX,
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

static AngleTurnControl gAngleTurn;
static const AngleTurnControl_Config gAngleTurnConfig = {
    .car = &gCar,
    /*
     * With the module horizontal and +Z upward, left/CCW yaw is positive.
     * Change to -1 if a manual left turn makes the reported Y angle decrease.
     */
    .leftTurnYawSign = ANGLE_TURN_LEFT_YAW_SIGN,
    .defaultCruiseRpm = ANGLE_TURN_DEFAULT_MAX_RPM,
    .creepRpm = 40,
    .brakeRateGain = 0.10f,
    .brakeAheadMaxDegrees = 12.0f,
    /* Was 2.0 deg and caused a repeatable ~88 deg result on 90 deg commands. */
    .angleToleranceDegrees = 0.4f,
    .stoppedRateToleranceDps = 6.0f,
    .settleSamples = 15U,
    .timeoutSamples = 1500U,
};

static volatile char gUartCommand[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t gUartCommandLength;
static volatile bool gUartCommandReady;
static volatile bool gMpu6050SampleDue;
static bool gMpu6050Ready;
static bool gGrayscaleStreamEnabled;
static uint8_t gGrayscaleStreamDivider;

static bool gLineTrackingEnabled;
static bool gLineTrackingDebugEnabled;
static int16_t gLineTrackingBaseSpeedRpm;
static float gLineTrackingIntegral;
static int16_t gLineTrackingLastErr;
static uint8_t gLineTrackingDebugDivider;
static uint8_t gLineFilteredValues[GRAYSCALE_SENSOR_CHANNELS];
static uint8_t gLinePendingValues[GRAYSCALE_SENSOR_CHANNELS];
static uint8_t gLinePendingCount[GRAYSCALE_SENSOR_CHANNELS];
static bool gLineFilterReady;

static bool gMotorStatusStreamEnabled;
static bool gMotorStatusReportOnce;

static void UART_sendString(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) *text);
        text++;
    }
}

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

static void UART_sendHex8(uint8_t value)
{
    static const char hexDigits[] = "0123456789ABCDEF";

    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[(value >> 4) & 0x0FU]);
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) hexDigits[value & 0x0FU]);
}

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
    UART_sendString("[Status]\r\n");
    UART_sendString("  Y / Y0      yaw read / reset\r\n");
    UART_sendString("  G / G1 / G0 grayscale once/stream/off\r\n");
    UART_sendString("  M / M1 / M0 motor status once/stream/off\r\n");
    UART_sendString("  H or ?      this help\r\n");
    UART_sendString("=====================================\r\n");
}

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

static void LineTracking_resetPid(void)
{
    uint8_t i;

    gLineTrackingIntegral = 0.0f;
    gLineTrackingLastErr = 0;
    gLineTrackingDebugDivider = 0U;
    gLineFilterReady = false;
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        gLineFilteredValues[i] = 0U;
        gLinePendingValues[i] = 0U;
        gLinePendingCount[i] = 0U;
    }
}

/*
 * Debounce digital chatter (especially X4/X5 on the line edge).
 * A channel value only flips after N identical new samples in a row.
 */
static void LineTracking_filterSensors(
    const uint8_t raw[GRAYSCALE_SENSOR_CHANNELS],
    uint8_t filtered[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t i;

    if (!gLineFilterReady) {
        for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
            gLineFilteredValues[i] = raw[i];
            gLinePendingValues[i] = raw[i];
            gLinePendingCount[i] = 0U;
            filtered[i] = raw[i];
        }
        gLineFilterReady = true;
        return;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (raw[i] == gLineFilteredValues[i]) {
            gLinePendingValues[i] = raw[i];
            gLinePendingCount[i] = 0U;
        } else if (raw[i] == gLinePendingValues[i]) {
            if (gLinePendingCount[i] < 255U) {
                gLinePendingCount[i]++;
            }
            if (gLinePendingCount[i] >= LINE_SENSOR_DEBOUNCE_SAMPLES) {
                gLineFilteredValues[i] = raw[i];
                gLinePendingCount[i] = 0U;
            }
        } else {
            gLinePendingValues[i] = raw[i];
            gLinePendingCount[i] = 1U;
        }
        filtered[i] = gLineFilteredValues[i];
    }
}

static int16_t LineTracking_computeErr(
    const uint8_t values[GRAYSCALE_SENSOR_CHANNELS])
{
    /*
     * Weights for X1..X8: left negative, right positive.
     * X4/X5 are the center pair: weight 0 so single-channel flicker
     * on the line edge does not create ±err and left/right hunting.
     */
    static const int16_t weights[GRAYSCALE_SENSOR_CHANNELS] = {
        -30, -20, -15, 0, 0, 15, 20, 30
    };

    int32_t weightedSum = 0;
    uint8_t activeCount = 0U;
    uint8_t outerActive = 0U;
    uint8_t centerActive = 0U;
    uint8_t i;
    int16_t err;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (values[i] == LINE_TRACKING_ACTIVE_LEVEL) {
            weightedSum += (int32_t) weights[i];
            activeCount++;
            if ((i == 3U) || (i == 4U)) {
                centerActive++;
            } else {
                outerActive++;
            }
        }
    }

    /* Only center sensors on the line => treat as perfectly centered. */
    if ((centerActive > 0U) && (outerActive == 0U)) {
        return 0;
    }

    if (activeCount > 0U) {
        err = (int16_t) (weightedSum / (int32_t) activeCount);
        if ((err <= LINE_ERR_DEADBAND) && (err >= -LINE_ERR_DEADBAND)) {
            return 0;
        }
        return err;
    }

    /* Lost line: keep turning toward last known direction. */
    if (gLineTrackingLastErr > 0) {
        return LINE_ERR_ABS_FALLBACK;
    }
    if (gLineTrackingLastErr < 0) {
        return (int16_t) (-LINE_ERR_ABS_FALLBACK);
    }
    return 0;
}

static void LineTracking_applyMotion(int16_t pidRpmOffset)
{
    int16_t speedRpm = gLineTrackingBaseSpeedRpm;
    int16_t pidAbs = (pidRpmOffset < 0) ? (int16_t) (-pidRpmOffset)
                                         : pidRpmOffset;
    int16_t innerRpm;
    uint8_t innerPercent;
    CarControl_Motion motion;

    if (speedRpm <= 0) {
        CarControl_stop(&gCar);
        return;
    }

    if (pidAbs <= LINE_PID_DEADBAND_RPM_OFFSET) {
        motion = CAR_CONTROL_FORWARD;
        CarControl_setMotion(&gCar, motion, speedRpm,
            CarControl_getTurnInnerPercent(&gCar));
        return;
    }

    /* Convert PID output magnitude into inner-wheel speed reduction. */
    if (pidAbs > speedRpm) {
        pidAbs = speedRpm;
    }
    innerRpm = (int16_t) (speedRpm - pidAbs);
    if (innerRpm < 0) {
        innerRpm = 0;
    }

    innerPercent = (uint8_t) ((int32_t) innerRpm * 100 /
        (int32_t) speedRpm);
    if (innerPercent > 100U) {
        innerPercent = 100U;
    }
    if (innerPercent == 0U) {
        innerPercent = 1U; /* keep non-zero to avoid deadband sticking */
    }

    /* pid sign: negative => line left => turn left */
    motion = (pidRpmOffset < 0) ? CAR_CONTROL_FORWARD_LEFT
                                 : CAR_CONTROL_FORWARD_RIGHT;
    CarControl_setMotion(&gCar, motion, speedRpm, innerPercent);
}

static void LineTracking_update(void)
{
    uint8_t rawValues[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t values[GRAYSCALE_SENSOR_CHANNELS];
    int16_t err;
    float pid;
    float derivative;
    int16_t pidOffsetRpm;
    uint8_t activeCount = 0U;
    uint8_t i;

    /* Read sensors (X1..X8), then debounce chatter. */
    Grayscale_Sensor_ReadAll(rawValues);
    LineTracking_filterSensors(rawValues, values);
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (values[i] == LINE_TRACKING_ACTIVE_LEVEL) {
            activeCount++;
        }
    }

    err = LineTracking_computeErr(values);
    if ((activeCount == 0U) || (err == 0)) {
        /* Prevent windup when centered or line is lost. */
        gLineTrackingIntegral = 0.0f;
    } else {
        gLineTrackingIntegral += (float) err;
        if (gLineTrackingIntegral > LINE_PID_INTEGRAL_LIMIT) {
            gLineTrackingIntegral = LINE_PID_INTEGRAL_LIMIT;
        } else if (gLineTrackingIntegral < -LINE_PID_INTEGRAL_LIMIT) {
            gLineTrackingIntegral = -LINE_PID_INTEGRAL_LIMIT;
        }
    }

    derivative = (float) (err - gLineTrackingLastErr);
    pid = (LINE_PID_KP * (float) err) +
        (LINE_PID_KI * gLineTrackingIntegral) +
        (LINE_PID_KD * derivative);

    /* Clamp pid to keep within speed reduction range. */
    {
        int16_t maxOffset = gLineTrackingBaseSpeedRpm;
        if (maxOffset < 0) {
            maxOffset = 0;
        }
        if (pid > (float) maxOffset) {
            pid = (float) maxOffset;
        } else if (pid < -(float) maxOffset) {
            pid = -(float) maxOffset;
        }
    }

    pidOffsetRpm = (int16_t) pid;
    gLineTrackingLastErr = err;
    LineTracking_applyMotion(pidOffsetRpm);

    if (gLineTrackingDebugEnabled) {
        gLineTrackingDebugDivider++;
        if (gLineTrackingDebugDivider >=
            LINE_DEBUG_PRINT_PERIOD_SAMPLES) {
            gLineTrackingDebugDivider = 0U;
            UART_sendString("LINE dbg err=");
            UART_sendInt32((int32_t) err);
            UART_sendString(" pid=");
            UART_sendInt32((int32_t) pidOffsetRpm);
            UART_sendString(" ");
            UART_sendString("GRAY");
            for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
                UART_sendString(" X");
                UART_sendInt32((int32_t) (i + 1U));
                DL_UART_Main_transmitDataBlocking(
                    UART_0_INST, (uint8_t) ':');
                UART_sendInt32((int32_t) values[i]);
            }
            UART_sendString("\r\n");
        }
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
    if ((command >= 'a') && (command <= 'z')) {
        command = (char) (command - ('a' - 'A'));
    }
    index++;

    if ((command == 'H') || (command == '?')) {
        UART_printHelp();
        return;
    }

    /* Manual motion commands cancel line tracking. */
    if (gLineTrackingEnabled && (command != 'I') &&
        ((command == 'X') || (command == 'T') ||
         (command == 'F') || (command == 'B') ||
         (command == 'L') || (command == 'R') ||
         (command == 'W'))) {
        gLineTrackingEnabled = false;
        gLineTrackingDebugEnabled = false;
        LineTracking_resetPid();
    }

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
        AngleTurnControl_cancel(&gAngleTurn);
        CarControl_stop(&gCar);
        gLineTrackingEnabled = false;
        gLineTrackingDebugEnabled = false;
        LineTracking_resetPid();
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
                gLineTrackingEnabled = false;
                gLineTrackingDebugEnabled = false;
                LineTracking_resetPid();
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
            gLineTrackingBaseSpeedRpm = (int16_t) newSpeed;
        }
        if (gLineTrackingBaseSpeedRpm < 1) {
            gLineTrackingBaseSpeedRpm = CAR_DEFAULT_SPEED_RPM;
        }

        gLineTrackingEnabled = true;
        gLineTrackingDebugEnabled = debug;
        LineTracking_resetPid();
        AngleTurnControl_cancel(&gAngleTurn);
        CarControl_stop(&gCar);

        UART_sendString("LINE_STARTED speed=");
        UART_sendInt32((int32_t) gLineTrackingBaseSpeedRpm);
        UART_sendString(debug ? " debug=1\r\n" : " debug=0\r\n");
        return;
    }
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
    SYSCFG_DL_init();

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
    AngleTurnControl_init(&gAngleTurn, &gAngleTurnConfig);
    Grayscale_Sensor_Init();

    gLineTrackingEnabled = false;
    gLineTrackingDebugEnabled = false;
    gLineTrackingBaseSpeedRpm = CAR_DEFAULT_SPEED_RPM;
    LineTracking_resetPid();

    UART_printBanner();
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
        "GRAYSCALE OK  AD0=PB11 AD1=PB5 AD2=PA1 OUT=PA14\r\n");
    UART_sendString("Ready. Send H for commands.\r\n");

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    DL_TimerA_startCounter(TIMER_PID_INST);

    while (1) {
        if (gMpu6050SampleDue) {
            AngleTurnControl_Result angleTurnResult =
                ANGLE_TURN_RESULT_NONE;

            gMpu6050SampleDue = false;
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

            if (gGrayscaleStreamEnabled) {
                gGrayscaleStreamDivider++;
                if (gGrayscaleStreamDivider >=
                    GRAYSCALE_STREAM_PERIOD_SAMPLES) {
                    gGrayscaleStreamDivider = 0U;
                    UART_reportGrayscale();
                }
            }

            if (gLineTrackingEnabled) {
                LineTracking_update();
            }
        }

        if (gUartCommandReady) {
            Car_processUartCommand();
            gUartCommandReady = false;
        }

        UART_serviceMotorStatus();
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpioBInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, GPIO_MOTOR_A_EB_1_PIN | GPIO_MOTOR_B_EB_2_PIN);
    uint32_t gpioAInterruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, GPIO_MOTOR_C_EB_3_PIN | GPIO_MOTOR_D_EB_4_PIN);

    if ((gpioBInterruptStatus & GPIO_MOTOR_A_EB_1_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorA);
    }
    if ((gpioBInterruptStatus & GPIO_MOTOR_B_EB_2_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorB);
    }
    if ((gpioAInterruptStatus & GPIO_MOTOR_C_EB_3_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorC);
    }
    if ((gpioAInterruptStatus & GPIO_MOTOR_D_EB_4_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorD);
    }

    DL_GPIO_clearInterruptStatus(GPIOB, gpioBInterruptStatus);
    DL_GPIO_clearInterruptStatus(GPIOA, gpioAInterruptStatus);
}

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_PID_INST)) {
        case DL_TIMER_IIDX_ZERO:
            MotorControl_update(&gMotorA);
            MotorControl_update(&gMotorB);
            MotorControl_update(&gMotorC);
            MotorControl_update(&gMotorD);
            gMpu6050SampleDue = true;
            break;
        default:
            break;
    }
}

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
