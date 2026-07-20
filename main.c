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

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_TARGET_RPM (1000U)
#define UART_COMMAND_BUFFER_SIZE (32U)
#define CAR_DEFAULT_SPEED_RPM (200)
#define CAR_DEFAULT_TURN_INNER_PERCENT (50U)
#define ENABLE_MOTOR_STATUS_UART (0)
#define ANGLE_TURN_DEFAULT_MAX_RPM (100)
#define ANGLE_TURN_LEFT_YAW_SIGN (1)

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
    .creepRpm = 45,
    .brakeRateGain = 0.12f,
    .brakeAheadMaxDegrees = 14.0f,
    .angleToleranceDegrees = 2.0f,
    .stoppedRateToleranceDps = 8.0f,
    /* Brief coast after entering the tolerance window. */
    .settleSamples = 20U,
    .timeoutSamples = 1500U,
};

static volatile char gUartCommand[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t gUartCommandLength;
static volatile bool gUartCommandReady;
static volatile bool gMpu6050SampleDue;
static bool gMpu6050Ready;

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

#if ENABLE_MOTOR_STATUS_UART
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
#endif

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
    if ((command >= 'a') && (command <= 'z')) {
        command = (char) (command - ('a' - 'A'));
    }
    index++;

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
        UART_sendString("STOPPED\r\n");
        return;
    }
    if (command == 'T') {
        if ((turnCommand == '\0') ||
            !Car_parseAngleTurnParameters(
                index, &angleTurnDegrees, &angleTurnMaximumRpm)) {
            UART_sendString("ANGLE_TURN_FORMAT_ERROR\r\n");
            return;
        }
        if (!gMpu6050Ready) {
            UART_sendString("MPU6050_ERROR\r\n");
            return;
        }
        if (AngleTurnControl_start(&gAngleTurn,
                turnCommand == 'L', angleTurnDegrees,
                angleTurnMaximumRpm)) {
            UART_sendString("ANGLE_TURN_STARTED\r\n");
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
            }
            return;
        }
        if (gUartCommand[index] == '\0') {
            UART_reportZAngle();
        }
        return;
    }
    if ((command != 'F') && (command != 'B') &&
        (command != 'L') && (command != 'R')) {
        return;
    }
    if (!Car_parseCommandParameters(index, &speedRpm, &speedSpecified,
            &turnInnerPercent, &turnSpecified)) {
        return;
    }
    if (turnSpecified && (turnCommand == '\0')) {
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
            return;
    }

    AngleTurnControl_cancel(&gAngleTurn);
    CarControl_setMotion(&gCar, motion, speedRpm, turnInnerPercent);
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

    UART_sendString(
        "\r\nMPU6050: keep car still for 5 seconds (calibrating)...\r\n");
    gMpu6050Ready = MPU6050_Angle_init();
    if (gMpu6050Ready) {
        UART_sendString("MPU6050 ready, WHO_AM_I=0x");
        UART_sendHex8((uint8_t) MPU6050_Angle_getDeviceId());
        UART_sendString(". Send Y to read angle, Y0 to reset.\r\n");
    } else {
        UART_sendString("MPU6050 init failed.\r\n");
    }

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
        }

        if (gUartCommandReady) {
            Car_processUartCommand();
            gUartCommandReady = false;
        }

        /* Motor status UART output temporarily disabled. */
#if ENABLE_MOTOR_STATUS_UART
        {
            static MotorControl_Status motorAStatus;
            static MotorControl_Status motorBStatus;
            static MotorControl_Status motorCStatus;
            static MotorControl_Status motorDStatus;
            static bool motorAStatusReady;
            static bool motorBStatusReady;
            static bool motorCStatusReady;
            static bool motorDStatusReady;

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
            if (motorAStatusReady && motorBStatusReady &&
                motorCStatusReady && motorDStatusReady) {
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
            }
        }
#endif
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
