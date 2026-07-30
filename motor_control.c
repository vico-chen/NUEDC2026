#include "motor_control.h"

static uint32_t MotorControl_getCountsPerOutputRevolution(
    const MotorControl *motor)
{
    return (uint32_t) motor->config.encoderPpr *
           (uint32_t) motor->config.gearRatio *
           (uint32_t) motor->config.encoderDecodeMultiplier;
}

static void MotorControl_resetPid(MotorControl *motor)
{
    motor->pidOutput = 0.0f;
    motor->pidLastError = 0.0f;
    motor->pidPreviousError = 0.0f;
}

static float MotorControl_absFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t MotorControl_roundOutput(float output)
{
    return (output >= 0.0f) ? (int16_t) (output + 0.5f)
                            : (int16_t) (output - 0.5f);
}

static void MotorControl_applyOutput(
    MotorControl *motor, int16_t pwmPercent)
{
    uint32_t compareValue;
    uint16_t pwmMagnitude;
    uint32_t loadValue = (uint32_t) motor->config.pwmPeriodCounts - 1U;

    if (pwmPercent > 100) {
        pwmPercent = 100;
    } else if (pwmPercent < -100) {
        pwmPercent = -100;
    }

    if (pwmPercent == 0) {
        DL_GPIO_clearPins(motor->config.directionIn1Port,
            motor->config.directionIn1Pin);
        DL_GPIO_clearPins(motor->config.directionIn2Port,
            motor->config.directionIn2Pin);
        compareValue = loadValue;
    } else {
        if (pwmPercent > 0) {
            DL_GPIO_setPins(motor->config.directionIn1Port,
                motor->config.directionIn1Pin);
            DL_GPIO_clearPins(motor->config.directionIn2Port,
                motor->config.directionIn2Pin);
            pwmMagnitude = (uint16_t) pwmPercent;
        } else {
            DL_GPIO_clearPins(motor->config.directionIn1Port,
                motor->config.directionIn1Pin);
            DL_GPIO_setPins(motor->config.directionIn2Port,
                motor->config.directionIn2Pin);
            pwmMagnitude = (uint16_t) (-pwmPercent);
        }

        compareValue =
            ((uint32_t) motor->config.pwmPeriodCounts *
                (100U - pwmMagnitude)) /
            100U;
        if (compareValue > loadValue) {
            compareValue = loadValue;
        }
    }

    DL_Timer_setCaptureCompareValue(motor->config.pwmInstance,
        compareValue, motor->config.pwmChannel);
}

static int16_t MotorControl_updatePid(MotorControl *motor,
    float measuredCountsPerSample, int16_t targetRpm)
{
    int8_t requestedDirection = 0;
    float targetCountsPerSample;
    float error;
    float increment;
    float outputMinimum;
    float outputMaximum;

    if (motor->coastMode) {
        MotorControl_resetPid(motor);
        motor->pidDirection = 0;
        return 0;
    }

    if (targetRpm != 0) {
        requestedDirection = (targetRpm > 0) ? 1 : -1;
        if (requestedDirection != motor->pidDirection) {
            MotorControl_resetPid(motor);
            motor->pidDirection = requestedDirection;
        }
    } else if (MotorControl_absFloat(measuredCountsPerSample) <=
               (float) motor->config.zeroSpeedDeadbandCounts) {
        /*
         * Stop braking once the wheel is inside the encoder deadband.
         * This prevents a residual integral term from reversing a stopped
         * wheel.
         */
        MotorControl_resetPid(motor);
        motor->pidDirection = 0;
        return 0;
    }

    targetCountsPerSample =
        ((float) targetRpm *
            (float) MotorControl_getCountsPerOutputRevolution(motor)) /
        (60.0f * (float) motor->config.sampleRateHz);
    error = targetCountsPerSample - measuredCountsPerSample;

    increment =
        motor->config.kp * (error - motor->pidLastError) +
        motor->config.ki * error +
        motor->config.kd *
            (error + motor->pidPreviousError -
                    (2.0f * motor->pidLastError));
    motor->pidOutput += increment;

    /*
     * Use signed control effort.  If a wheel is mechanically back-driven
     * above its target (common for the inner wheels of a tight arc), the
     * controller may briefly command the opposite bridge direction to
     * generate braking torque.  Limit that counter-torque independently so
     * it cannot become an uncontrolled reversal.
     */
    if (targetRpm > 0) {
        outputMinimum = -motor->config.brakeMaxPercent;
        outputMaximum = motor->config.outputMaxPercent;
    } else if (targetRpm < 0) {
        outputMinimum = -motor->config.outputMaxPercent;
        outputMaximum = motor->config.brakeMaxPercent;
    } else {
        outputMinimum = -motor->config.brakeMaxPercent;
        outputMaximum = motor->config.brakeMaxPercent;
    }

    if (motor->pidOutput > outputMaximum) {
        motor->pidOutput = outputMaximum;
    } else if (motor->pidOutput < outputMinimum) {
        motor->pidOutput = outputMinimum;
    }

    motor->pidPreviousError = motor->pidLastError;
    motor->pidLastError = error;

    return MotorControl_roundOutput(motor->pidOutput);
}

void MotorControl_init(
    MotorControl *motor, const MotorControl_Config *config)
{
    *motor = (MotorControl) {0};
    motor->config = *config;
    MotorControl_applyOutput(motor, 0);
    DL_Timer_startCounter(motor->config.pwmInstance);
}

void MotorControl_setTargetRpm(MotorControl *motor, int16_t targetRpm)
{
    if (targetRpm > motor->config.maxTargetRpm) {
        targetRpm = motor->config.maxTargetRpm;
    } else if (targetRpm < -motor->config.maxTargetRpm) {
        targetRpm = -motor->config.maxTargetRpm;
    }
    motor->coastMode = false;
    motor->targetRpm = targetRpm;
}

void MotorControl_coast(MotorControl *motor)
{
    motor->targetRpm = 0;
    motor->coastMode = true;
    motor->pidDirection = 0;
    MotorControl_resetPid(motor);
    MotorControl_applyOutput(motor, 0);
}

void MotorControl_handleEncoderEdge(MotorControl *motor)
{
    bool phaseB = (DL_GPIO_readPins(motor->config.encoderPhaseBPort,
                       motor->config.encoderPhaseBPin) != 0U);
    bool phaseA = (DL_GPIO_readPins(motor->config.encoderPhaseAPort,
                       motor->config.encoderPhaseAPin) != 0U);

    if (phaseB) {
        motor->encoderCount += phaseA ? 1 : -1;
    } else {
        motor->encoderCount += phaseA ? -1 : 1;
    }
}

void MotorControl_update(MotorControl *motor)
{
    float alpha = motor->config.speedFilterAlpha;

    motor->speedCountsPerSample = motor->encoderCount;
    motor->encoderCount = 0;

    if ((alpha <= 0.0f) || (alpha > 1.0f)) {
        alpha = 1.0f;
    }
    if (!motor->speedFilterReady) {
        motor->filteredSpeedCountsPerSample =
            (float) motor->speedCountsPerSample;
        motor->speedFilterReady = true;
    } else {
        motor->filteredSpeedCountsPerSample += alpha *
            ((float) motor->speedCountsPerSample -
                motor->filteredSpeedCountsPerSample);
    }

    motor->pwmPercent = MotorControl_updatePid(
        motor, motor->filteredSpeedCountsPerSample, motor->targetRpm);
    MotorControl_applyOutput(motor, motor->pwmPercent);

    motor->reportCountAccumulator += motor->speedCountsPerSample;
    motor->reportDivider++;
    if (motor->reportDivider >= motor->config.reportSamples) {
        motor->reportDivider = 0U;
        motor->reportCounts = motor->reportCountAccumulator;
        motor->reportCountAccumulator = 0;
        motor->statusReady = true;
    }
}

bool MotorControl_takeStatus(
    MotorControl *motor, MotorControl_Status *status)
{
    int64_t rpmTimes10Numerator;
    int64_t rpmTimes10Denominator;
    uint32_t countsPerOutputRevolution;

    if (!motor->statusReady) {
        return false;
    }

    status->targetRpm = motor->targetRpm;
    status->encoderCounts = motor->reportCounts;
    status->pwmPercent = motor->pwmPercent;
    motor->statusReady = false;

    countsPerOutputRevolution =
        MotorControl_getCountsPerOutputRevolution(motor);
    rpmTimes10Numerator =
        (int64_t) status->encoderCounts * 600LL *
        (int64_t) motor->config.sampleRateHz;
    rpmTimes10Denominator =
        (int64_t) motor->config.reportSamples *
        (int64_t) countsPerOutputRevolution;

    if (rpmTimes10Numerator >= 0) {
        rpmTimes10Numerator += rpmTimes10Denominator / 2;
    } else {
        rpmTimes10Numerator -= rpmTimes10Denominator / 2;
    }
    status->speedRpmTimes10 =
        (int32_t) (rpmTimes10Numerator / rpmTimes10Denominator);

    return true;
}
