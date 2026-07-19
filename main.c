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
#define MOTOR_COUNT (2U)

static MotorControl gMotorA;
static const MotorControl_Config gMotorAConfig = {
    .pwmInstance = PWM_MOTOR_AB_INST,
    .pwmChannel = DL_TIMER_CC_0_INDEX,
    .directionPort = GPIO_MOTOR_A_PORT,
    .directionIn1Pin = GPIO_MOTOR_A_AIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_A_AIN_2_PIN,
    .encoderPort = GPIO_MOTOR_A_PORT,
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
    .directionPort = GPIO_MOTOR_B_PORT,
    .directionIn1Pin = GPIO_MOTOR_B_BIN_1_PIN,
    .directionIn2Pin = GPIO_MOTOR_B_BIN_2_PIN,
    .encoderPort = GPIO_MOTOR_B_PORT,
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

static volatile int16_t gRxTargets[MOTOR_COUNT];
static volatile uint16_t gRxValue;
static volatile uint8_t gRxDigitCount;
static volatile uint8_t gRxMotorIndex;
static volatile bool gRxInvalid;
static volatile bool gRxNegative;

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

static void UART_finishTargetValue(void)
{
    if ((gRxDigitCount == 0U) || (gRxMotorIndex >= MOTOR_COUNT)) {
        gRxInvalid = true;
        return;
    }

    gRxTargets[gRxMotorIndex] = gRxNegative ?
        -(int16_t) gRxValue : (int16_t) gRxValue;
    gRxMotorIndex++;
    gRxValue = 0U;
    gRxDigitCount = 0U;
    gRxNegative = false;
}

static void UART_resetCommand(void)
{
    gRxValue = 0U;
    gRxDigitCount = 0U;
    gRxMotorIndex = 0U;
    gRxInvalid = false;
    gRxNegative = false;
}

int main(void)
{
    SYSCFG_DL_init();

    /*
     * TB6612 motor A: PA12/PB6/PB7, encoder PB0/PB16.
     * TB6612 motor B: PA13/PB8/PB15, encoder PB17/PB12.
     *
     * UART "<motorA_rpm> <motorB_rpm>" + CR/LF sets both target RPM.
     * Target 0 actively brakes to the deadband, then short-brakes.
     * Positive runs forward and negative reverses.
     */
    MotorControl_init(&gMotorA, &gMotorAConfig);
    MotorControl_init(&gMotorB, &gMotorBConfig);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    DL_TimerA_startCounter(TIMER_PID_INST);

    while (1) {
        static MotorControl_Status motorAStatus;
        static MotorControl_Status motorBStatus;
        static bool motorAStatusReady;
        static bool motorBStatusReady;

        if (MotorControl_takeStatus(&gMotorA, &motorAStatus)) {
            motorAStatusReady = true;
        }
        if (MotorControl_takeStatus(&gMotorB, &motorBStatus)) {
            motorBStatusReady = true;
        }
        if (motorAStatusReady && motorBStatusReady) {
            UART_reportMotorStatus('A', &motorAStatus);
            UART_sendString(",");
            UART_reportMotorStatus('B', &motorBStatus);
            UART_sendString("\r\n");
            motorAStatusReady = false;
            motorBStatusReady = false;
        }
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t interruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, GPIO_MOTOR_A_EB_1_PIN | GPIO_MOTOR_B_EB_2_PIN);

    if ((interruptStatus & GPIO_MOTOR_A_EB_1_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorA);
    }
    if ((interruptStatus & GPIO_MOTOR_B_EB_2_PIN) != 0U) {
        MotorControl_handleEncoderEdge(&gMotorB);
    }

    DL_GPIO_clearInterruptStatus(GPIOB, interruptStatus);
}

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_PID_INST)) {
        case DL_TIMER_IIDX_ZERO:
            MotorControl_update(&gMotorA);
            MotorControl_update(&gMotorB);
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

            if (gRxInvalid &&
                (rxData != (uint8_t) '\r') &&
                (rxData != (uint8_t) '\n')) {
                break;
            } else if ((rxData == (uint8_t) '-') &&
                       (gRxDigitCount == 0U) && !gRxNegative) {
                gRxNegative = true;
            } else if ((rxData >= (uint8_t) '0') &&
                       (rxData <= (uint8_t) '9')) {
                uint16_t newValue =
                    (gRxValue * 10U) + (uint16_t) (rxData - (uint8_t) '0');

                gRxDigitCount++;
                if ((gRxDigitCount > 4U) ||
                    (newValue > MOTOR_MAX_TARGET_RPM)) {
                    gRxInvalid = true;
                } else {
                    gRxValue = newValue;
                }
            } else if ((rxData == (uint8_t) ' ') ||
                       (rxData == (uint8_t) '\t')) {
                if (gRxDigitCount > 0U) {
                    UART_finishTargetValue();
                } else if (gRxNegative) {
                    gRxInvalid = true;
                }
            } else if ((rxData == (uint8_t) '\r') ||
                       (rxData == (uint8_t) '\n')) {
                if (!gRxInvalid && (gRxDigitCount > 0U)) {
                    UART_finishTargetValue();
                } else if (gRxNegative) {
                    gRxInvalid = true;
                }

                if (!gRxInvalid && (gRxMotorIndex == MOTOR_COUNT)) {
                    MotorControl_setTargetRpm(&gMotorA, gRxTargets[0]);
                    MotorControl_setTargetRpm(&gMotorB, gRxTargets[1]);
                }
                UART_resetCommand();
            } else {
                gRxInvalid = true;
            }
            break;
        default:
            break;
    }
}
