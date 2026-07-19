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

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_TARGET_RPM (1000U)
#define UART_COMMAND_BUFFER_SIZE (32U)
#define CAR_DEFAULT_SPEED_RPM (200)

/*
 * Motors on the right side are mirrored on a typical four-wheel chassis.
 * Change an individual sign if that wheel rotates opposite to the command.
 */
#define MOTOR_A_FORWARD_SIGN (1)
#define MOTOR_B_FORWARD_SIGN (1)
#define MOTOR_C_FORWARD_SIGN (-1)
#define MOTOR_D_FORWARD_SIGN (-1)

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

static volatile char gUartCommand[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t gUartCommandLength;
static volatile bool gUartCommandReady;
static int16_t gCarSpeedRpm = CAR_DEFAULT_SPEED_RPM;

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

static void Car_setWheelRpm(int16_t leftRearRpm, int16_t leftFrontRpm,
    int16_t rightFrontRpm, int16_t rightRearRpm)
{
    MotorControl_setTargetRpm(
        &gMotorA, (int16_t) (leftRearRpm * MOTOR_A_FORWARD_SIGN));
    MotorControl_setTargetRpm(
        &gMotorB, (int16_t) (leftFrontRpm * MOTOR_B_FORWARD_SIGN));
    MotorControl_setTargetRpm(
        &gMotorC, (int16_t) (rightFrontRpm * MOTOR_C_FORWARD_SIGN));
    MotorControl_setTargetRpm(
        &gMotorD, (int16_t) (rightRearRpm * MOTOR_D_FORWARD_SIGN));
}

static bool Car_parseCommandSpeed(
    uint8_t startIndex, int16_t *speedRpm, bool *speedSpecified)
{
    uint8_t index = startIndex;
    uint16_t value = 0U;
    bool hasDigit = false;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }

    if (gUartCommand[index] == '\0') {
        *speedRpm = gCarSpeedRpm;
        *speedSpecified = false;
        return true;
    }

    while ((gUartCommand[index] >= '0') &&
           (gUartCommand[index] <= '9')) {
        value = (uint16_t) ((value * 10U) +
            (uint16_t) (gUartCommand[index] - '0'));
        hasDigit = true;
        index++;
        if (value > MOTOR_MAX_TARGET_RPM) {
            return false;
        }
    }

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    if (!hasDigit || (gUartCommand[index] != '\0')) {
        return false;
    }

    *speedRpm = (int16_t) value;
    *speedSpecified = true;
    return true;
}

static void Car_processUartCommand(void)
{
    uint8_t index = 0U;
    char command;
    int16_t speedRpm;
    bool speedSpecified;

    while ((gUartCommand[index] == ' ') ||
           (gUartCommand[index] == '\t')) {
        index++;
    }
    command = gUartCommand[index];
    if ((command >= 'a') && (command <= 'z')) {
        command = (char) (command - ('a' - 'A'));
    }
    index++;

    if (command == 'X') {
        Car_setWheelRpm(0, 0, 0, 0);
        return;
    }
    if ((command != 'W') && (command != 'S') &&
        (command != 'A') && (command != 'D')) {
        return;
    }
    if (!Car_parseCommandSpeed(index, &speedRpm, &speedSpecified)) {
        return;
    }
    if (speedSpecified) {
        gCarSpeedRpm = speedRpm;
    }

    switch (command) {
        case 'W':
            Car_setWheelRpm(speedRpm, speedRpm, speedRpm, speedRpm);
            break;
        case 'S':
            Car_setWheelRpm(-speedRpm, -speedRpm, -speedRpm, -speedRpm);
            break;
        case 'A':
            Car_setWheelRpm(-speedRpm, -speedRpm, speedRpm, speedRpm);
            break;
        case 'D':
            Car_setWheelRpm(speedRpm, speedRpm, -speedRpm, -speedRpm);
            break;
        default:
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    /*
     * TB6612 motor A: PA12/PB6/PB7, encoder PB0/PB16.
     * TB6612 motor B: PA13/PB8/PB15, encoder PB17/PB12.
     * TB6612 motor C: PA8/PA26/PB24, encoder PB9/PA27.
     * TB6612 motor D: PA22/PB18/PA18, encoder PA24/PA17.
     *
     * UART remote control: W/S/A/D [rpm], X stops the car.
     * Target 0 actively brakes to the deadband, then short-brakes.
     */
    MotorControl_init(&gMotorA, &gMotorAConfig);
    MotorControl_init(&gMotorB, &gMotorBConfig);
    MotorControl_init(&gMotorC, &gMotorCConfig);
    MotorControl_init(&gMotorD, &gMotorDConfig);

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
        static MotorControl_Status motorAStatus;
        static MotorControl_Status motorBStatus;
        static MotorControl_Status motorCStatus;
        static MotorControl_Status motorDStatus;
        static bool motorAStatusReady;
        static bool motorBStatusReady;
        static bool motorCStatusReady;
        static bool motorDStatusReady;

        if (gUartCommandReady) {
            Car_processUartCommand();
            gUartCommandReady = false;
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
