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

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_PERIOD_COUNTS (100U)
/* Edge-aligned PWM programs LOAD as (period - 1). CC must stay within 0..LOAD. */
#define MOTOR_PWM_LOAD_VALUE (MOTOR_PWM_PERIOD_COUNTS - 1U)
#define MOTOR_INITIAL_SPEED_PERCENT (0U)
#define ENCODER_PPR (13U)
#define MOTOR_GEAR_RATIO (20U)
#define ENCODER_DECODE_MULTIPLIER (2U)
#define ENCODER_COUNTS_PER_OUTPUT_REVOLUTION \
    (ENCODER_PPR * MOTOR_GEAR_RATIO * ENCODER_DECODE_MULTIPLIER)
#define SPEED_REPORT_SAMPLES (10U)
#define PID_SAMPLE_RATE_HZ (100U)
#define PID_KP (5.0f)
#define PID_KI (1.0f)
#define PID_KD (0.2f)
#define PID_OUTPUT_MAX_PERCENT (99.0f)
#define MOTOR_MAX_TARGET_RPM (1000U)
#define ZERO_SPEED_THRESHOLD_COUNTS (1)
#define ZERO_BRAKE_KP (1.5f)
#define ZERO_BRAKE_MIN_PERCENT (8.0f)
#define ZERO_BRAKE_MAX_PERCENT (40.0f)

static volatile uint16_t gRxValue;
static volatile uint8_t gRxDigitCount;
static volatile bool gRxInvalid;
static volatile bool gRxNegative;
static volatile int16_t gTargetRpm;
static volatile int16_t gMotorPwmPercent;
static volatile int32_t gEncoderCount;
static volatile int32_t gSpeedCountsPer10ms;
static volatile int32_t gSpeedReportCountAccumulator;
static volatile int32_t gSpeedCountsPer100ms;
static volatile uint8_t gSpeedReportDivider;
static volatile bool gSpeedReportReady;
static float gPidOutput;
static float gPidLastError;
static float gPidPreviousError;
static int8_t gPidDirection;
static int8_t gZeroBrakeEncoderDirection;

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

static void UART_reportStatus(int32_t speedCountsPer100ms,
    int16_t targetRpm, int16_t pwmPercent)
{
    int32_t rpmTimes10Numerator = speedCountsPer100ms * 6000;
    int32_t rpmTimes10;
    uint32_t rpmMagnitude;

    /* Round to the nearest 0.1 RPM before integer division. */
    if (rpmTimes10Numerator >= 0) {
        rpmTimes10Numerator +=
            (int32_t) (ENCODER_COUNTS_PER_OUTPUT_REVOLUTION / 2U);
    } else {
        rpmTimes10Numerator -=
            (int32_t) (ENCODER_COUNTS_PER_OUTPUT_REVOLUTION / 2U);
    }
    rpmTimes10 = rpmTimes10Numerator /
                 (int32_t) ENCODER_COUNTS_PER_OUTPUT_REVOLUTION;

    UART_sendString("target_rpm=");
    UART_sendInt32((int32_t) targetRpm);
    UART_sendString(",speed_rpm=");
    if (rpmTimes10 < 0) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '-');
        rpmMagnitude = (uint32_t) (-rpmTimes10);
    } else {
        rpmMagnitude = (uint32_t) rpmTimes10;
    }
    UART_sendInt32((int32_t) (rpmMagnitude / 10U));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) '.');
    DL_UART_Main_transmitDataBlocking(
        UART_0_INST, (uint8_t) ('0' + (rpmMagnitude % 10U)));
    UART_sendString(",counts_100ms=");
    UART_sendInt32(speedCountsPer100ms);
    UART_sendString(",pwm_percent=");
    UART_sendInt32((int32_t) pwmPercent);
    UART_sendString("\r\n");
}

static void Motor_setSpeedPercent(int16_t speedPercent)
{
    uint32_t compareValue;
    uint16_t speedMagnitude;

    if (speedPercent > 100) {
        speedPercent = 100;
    } else if (speedPercent < -100) {
        speedPercent = -100;
    }

    if (speedPercent == 0) {
        /* IN1=IN2=1 selects TB6612 short-brake mode. */
        DL_GPIO_setPins(GPIO_MOTOR_A_PORT,
            GPIO_MOTOR_A_AIN_1_PIN | GPIO_MOTOR_A_AIN_2_PIN);
        compareValue = MOTOR_PWM_LOAD_VALUE;
    } else {
        if (speedPercent > 0) {
            DL_GPIO_setPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN_1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN_2_PIN);
            speedMagnitude = (uint16_t) speedPercent;
        } else {
            DL_GPIO_clearPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN_1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN_2_PIN);
            speedMagnitude = (uint16_t) (-speedPercent);
        }

        /*
         * Edge-aligned PWM: high on LOAD, low on compare-down.
         * duty% rises as compare falls; compare=0 -> ~100%, compare=LOAD -> ~0%.
         */
        compareValue =
            (MOTOR_PWM_PERIOD_COUNTS * (100U - speedMagnitude)) /
            100U;
        if (compareValue > MOTOR_PWM_LOAD_VALUE) {
            compareValue = MOTOR_PWM_LOAD_VALUE;
        }
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_A_INST, compareValue, DL_TIMER_CC_0_INDEX);
}

static void PID_reset(void)
{
    gPidOutput = 0.0f;
    gPidLastError = 0.0f;
    gPidPreviousError = 0.0f;
}

static int16_t PID_update(
    int32_t measuredCountsPer10ms, int16_t targetRpm)
{
    int8_t requestedDirection;
    int8_t measuredDirection;
    uint16_t targetMagnitudeRpm;
    float targetCountsPer10ms;
    float measuredCounts;
    float brakeOutput;
    float error;
    float increment;

    measuredCounts = (measuredCountsPer10ms < 0) ?
        (float) (-measuredCountsPer10ms) :
        (float) measuredCountsPer10ms;

    if (targetRpm == 0) {
        PID_reset();

        if ((gPidDirection == 0) ||
            (measuredCountsPer10ms >= -ZERO_SPEED_THRESHOLD_COUNTS &&
             measuredCountsPer10ms <= ZERO_SPEED_THRESHOLD_COUNTS)) {
            gZeroBrakeEncoderDirection = 0;
            return 0;
        }

        measuredDirection = (measuredCountsPer10ms > 0) ? 1 : -1;
        if (gZeroBrakeEncoderDirection == 0) {
            gZeroBrakeEncoderDirection = measuredDirection;
        } else if (measuredDirection != gZeroBrakeEncoderDirection) {
            /*
             * The shaft has crossed zero. Stop active reverse torque and
             * hold with short-brake instead of accelerating the other way.
             */
            gZeroBrakeEncoderDirection = 0;
            return 0;
        }

        brakeOutput = ZERO_BRAKE_KP * measuredCounts;
        if (brakeOutput > ZERO_BRAKE_MAX_PERCENT) {
            brakeOutput = ZERO_BRAKE_MAX_PERCENT;
        } else if (brakeOutput < ZERO_BRAKE_MIN_PERCENT) {
            brakeOutput = ZERO_BRAKE_MIN_PERCENT;
        }

        return (int16_t) (-gPidDirection *
            (int16_t) (brakeOutput + 0.5f));
    }

    requestedDirection = (targetRpm > 0) ? 1 : -1;
    targetMagnitudeRpm = (targetRpm > 0) ?
        (uint16_t) targetRpm : (uint16_t) (-targetRpm);

    if (requestedDirection != gPidDirection) {
        PID_reset();
        gPidDirection = requestedDirection;
    }
    gZeroBrakeEncoderDirection = 0;

    targetCountsPer10ms =
        ((float) targetMagnitudeRpm *
            (float) ENCODER_COUNTS_PER_OUTPUT_REVOLUTION) /
        (60.0f * (float) PID_SAMPLE_RATE_HZ);

    error = targetCountsPer10ms - measuredCounts;

    increment =
        PID_KP * (error - gPidLastError) +
        PID_KI * error +
        PID_KD *
            (error + gPidPreviousError - (2.0f * gPidLastError));
    gPidOutput += increment;

    if (gPidOutput > PID_OUTPUT_MAX_PERCENT) {
        gPidOutput = PID_OUTPUT_MAX_PERCENT;
    } else if (gPidOutput < 0.0f) {
        gPidOutput = 0.0f;
    }

    gPidPreviousError = gPidLastError;
    gPidLastError = error;

    return (int16_t) (requestedDirection *
        (int16_t) (gPidOutput + 0.5f));
}

int main(void)
{
    SYSCFG_DL_init();

    /*
     * TB6612 motor A:
     *   PA12 -> PWMA
     *   PB6  -> AIN1
     *   PB7  -> AIN2
     *
     * UART ASCII "-1000".."1000" + CR/LF sets target output-shaft RPM.
     * Target 0 short-brakes; positive runs forward and negative reverses.
     */
    Motor_setSpeedPercent(MOTOR_INITIAL_SPEED_PERCENT);
    DL_TimerG_startCounter(PWM_MOTOR_A_INST);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MOTOR_A_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MOTOR_A_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    DL_TimerA_startCounter(TIMER_PID_INST);

    while (1) {
        if (gSpeedReportReady) {
            int32_t speed = gSpeedCountsPer100ms;
            int16_t targetRpm = gTargetRpm;
            int16_t pwmPercent = gMotorPwmPercent;
            gSpeedReportReady = false;
            UART_reportStatus(speed, targetRpm, pwmPercent);
        }
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t interruptStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_EB_1_PIN);

    if ((interruptStatus & GPIO_MOTOR_A_EB_1_PIN) != 0U) {
        bool ebState = (DL_GPIO_readPins(
                            GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_EB_1_PIN) != 0U);
        bool eaState = (DL_GPIO_readPins(
                            GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_EA_1_PIN) != 0U);

        if (ebState) {
            gEncoderCount += eaState ? 1 : -1;
        } else {
            gEncoderCount += eaState ? -1 : 1;
        }

        DL_GPIO_clearInterruptStatus(
            GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_EB_1_PIN);
    }
}

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_PID_INST)) {
        case DL_TIMER_IIDX_ZERO:
        {
            gSpeedCountsPer10ms = gEncoderCount;
            gEncoderCount = 0;
            gMotorPwmPercent =
                PID_update(gSpeedCountsPer10ms, gTargetRpm);
            Motor_setSpeedPercent(gMotorPwmPercent);
            gSpeedReportCountAccumulator += gSpeedCountsPer10ms;

            gSpeedReportDivider++;
            if (gSpeedReportDivider >= SPEED_REPORT_SAMPLES) {
                gSpeedReportDivider = 0U;
                gSpeedCountsPer100ms = gSpeedReportCountAccumulator;
                gSpeedReportCountAccumulator = 0;
                gSpeedReportReady = true;
            }
            break;
        }
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

            if ((rxData == (uint8_t) '-') &&
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
            } else if ((rxData == (uint8_t) '\r') ||
                       (rxData == (uint8_t) '\n')) {
                if ((gRxDigitCount > 0U) && !gRxInvalid) {
                    gTargetRpm = gRxNegative ?
                        -(int16_t) gRxValue : (int16_t) gRxValue;
                }

                gRxValue      = 0U;
                gRxDigitCount = 0U;
                gRxInvalid    = false;
                gRxNegative   = false;
            } else {
                gRxValue      = 0U;
                gRxDigitCount = 0U;
                gRxInvalid    = false;
                gRxNegative   = false;
            }
            break;
        default:
            break;
    }
}
