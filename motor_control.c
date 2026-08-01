#include "motor_control.h"

/* 计算电机输出轴旋转一圈对应的编码器累计计数。 */
static uint32_t MotorControl_getCountsPerOutputRevolution(
    const MotorControl *motor)
{
    return (uint32_t) motor->config.encoderPpr *
           (uint32_t) motor->config.gearRatio *
           (uint32_t) motor->config.encoderDecodeMultiplier;
}

/* 将一个状态统计周期的编码器计数换算为 0.1 RPM。 */
static int32_t MotorControl_countsToRpmTimes10(
    const MotorControl *motor, int32_t encoderCounts)
{
    int64_t numerator;
    int64_t denominator;

    numerator = (int64_t) encoderCounts * 600LL *
        (int64_t) motor->config.sampleRateHz;
    denominator = (int64_t) motor->config.reportSamples *
        (int64_t) MotorControl_getCountsPerOutputRevolution(motor);

    if (denominator == 0LL) {
        return 0;
    }
    if (numerator >= 0LL) {
        numerator += denominator / 2LL;
    } else {
        numerator -= denominator / 2LL;
    }
    return (int32_t) (numerator / denominator);
}

/* 清除增量式 PID 历史量，避免切换方向时沿用旧输出。 */
static void MotorControl_resetPid(MotorControl *motor)
{
    motor->pidOutput = 0.0f;
    motor->pidLastError = 0.0f;
    motor->pidPreviousError = 0.0f;
}

/* 将带符号 PWM 百分比转换为 H 桥电平和定时器比较值。 */
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

/* 根据目标转速和本周期编码器计数计算 PWM 百分比。 */
static int16_t MotorControl_updatePid(MotorControl *motor,
    int32_t measuredCountsPerSample, int16_t targetRpm)
{
    int8_t requestedDirection;
    int16_t signedOutputPercent;
    uint16_t targetMagnitudeRpm;
    int32_t measuredMagnitude;
    float targetCountsPerSample;
    float measuredCounts;
    float error;
    float increment;

    if (targetRpm == 0) {
        if (motor->coastMode) {
            MotorControl_resetPid(motor);
            motor->zeroBrakeDirection = 0;
            motor->pidDirection = 0;
            return 0;
        }

        if (motor->pidDirection != 0) {
            motor->pidDirection = 0;
            MotorControl_resetPid(motor);
        }

        measuredMagnitude = (measuredCountsPerSample < 0) ?
            -measuredCountsPerSample : measuredCountsPerSample;

        if (measuredMagnitude <= motor->config.zeroSpeedDeadbandCounts) {
            MotorControl_resetPid(motor);
            motor->zeroBrakeDirection = 0;
            return 0;
        }

        /* 制动力始终与实测方向相反，不能沿用上一条运动命令。 */
        motor->zeroBrakeDirection = (measuredCountsPerSample > 0) ? -1 : 1;

        error = -(float) measuredMagnitude;
        increment =
            motor->config.kp * (error - motor->pidLastError) +
            motor->config.ki * error +
            motor->config.kd *
                (error + motor->pidPreviousError -
                    (2.0f * motor->pidLastError));
        motor->pidOutput += increment;

        if (motor->pidOutput <
            -motor->config.zeroSpeedBrakeMaxPercent) {
            motor->pidOutput =
                -motor->config.zeroSpeedBrakeMaxPercent;
        } else if (motor->pidOutput > 0.0f) {
            motor->pidOutput = 0.0f;
        }

        motor->pidPreviousError = motor->pidLastError;
        motor->pidLastError = error;

        return (int16_t) (motor->zeroBrakeDirection *
            (int16_t) ((-motor->pidOutput) + 0.5f));
    }

    requestedDirection = (targetRpm > 0) ? 1 : -1;
    targetMagnitudeRpm = (targetRpm > 0) ?
        (uint16_t) targetRpm : (uint16_t) (-targetRpm);

    if (requestedDirection != motor->pidDirection) {
        MotorControl_resetPid(motor);
        motor->pidDirection = requestedDirection;
        motor->zeroBrakeDirection = 0;
    }

    targetCountsPerSample =
        ((float) targetMagnitudeRpm *
            (float) MotorControl_getCountsPerOutputRevolution(motor)) /
        (60.0f * (float) motor->config.sampleRateHz);

    measuredCounts = (measuredCountsPerSample < 0) ?
        (float) (-measuredCountsPerSample) :
        (float) measuredCountsPerSample;
    error = targetCountsPerSample - measuredCounts;

    increment =
        motor->config.kp * (error - motor->pidLastError) +
        motor->config.ki * error +
        motor->config.kd *
            (error + motor->pidPreviousError -
                (2.0f * motor->pidLastError));
    motor->pidOutput += increment;

    /*
     * 非零目标转速下也允许有限反向输出。
     * 例如转弯内侧轮目标只有 10 RPM、但被车体拖到 90 RPM 时，
     * PID 可以反向制动，而不是把 PWM 降到 0 后任其继续滑行。
     * 反向制动力沿用零速制动上限，避免突然满功率反转。
     */
    if (motor->pidOutput > motor->config.outputMaxPercent) {
        motor->pidOutput = motor->config.outputMaxPercent;
    } else if (motor->pidOutput <
        -motor->config.zeroSpeedBrakeMaxPercent) {
        motor->pidOutput =
            -motor->config.zeroSpeedBrakeMaxPercent;
    }

    motor->pidPreviousError = motor->pidLastError;
    motor->pidLastError = error;

    /* 正负输出分别四舍五入，保留负号表示与目标方向相反的制动力。 */
    if (motor->pidOutput >= 0.0f) {
        signedOutputPercent =
            (int16_t) (motor->pidOutput + 0.5f);
    } else {
        signedOutputPercent =
            (int16_t) (motor->pidOutput - 0.5f);
    }

    return (int16_t) (requestedDirection * signedOutputPercent);
}

void MotorControl_init(
    MotorControl *motor, const MotorControl_Config *config)
{
    /* 先将输出置零，再启动 PWM 定时器。 */
    *motor = (MotorControl) {0};
    motor->config = *config;
    MotorControl_applyOutput(motor, 0);
    DL_Timer_startCounter(motor->config.pwmInstance);
}

void MotorControl_setTargetRpm(MotorControl *motor, int16_t targetRpm)
{
    /* 对外部命令限幅，防止目标转速超过配置上限。 */
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
    motor->zeroBrakeDirection = 0;
    motor->pidDirection = 0;
    MotorControl_resetPid(motor);
    MotorControl_applyOutput(motor, 0);
}

void MotorControl_handleEncoderEdge(MotorControl *motor)
{
    /* 在 B 相边沿读取 A/B 电平，通过正交关系判断计数方向。 */
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
    /* 每 10 ms 取走编码器计数、更新 PID 并写入 PWM。 */
    motor->speedCountsPerSample = motor->encoderCount;
    motor->encoderCount = 0;
    motor->pwmPercent = MotorControl_updatePid(
        motor, motor->speedCountsPerSample, motor->targetRpm);
    MotorControl_applyOutput(motor, motor->pwmPercent);

    motor->reportCountAccumulator += motor->speedCountsPerSample;
    motor->reportDivider++;
    if (motor->reportDivider >= motor->config.reportSamples) {
        /* 低频累计状态，避免调试串口占用过多主循环时间。 */
        motor->reportDivider = 0U;
        motor->reportCounts = motor->reportCountAccumulator;
        motor->reportCountAccumulator = 0;
        motor->statusReady = true;
    }
}

bool MotorControl_takeStatus(
    MotorControl *motor, MotorControl_Status *status)
{
    if (!motor->statusReady) {
        return false;
    }

    /* 状态只消费一次，随后将累计计数换算为 0.1 RPM。 */
    status->targetRpm = motor->targetRpm;
    status->encoderCounts = motor->reportCounts;
    status->pwmPercent = motor->pwmPercent;
    motor->statusReady = false;

    status->speedRpmTimes10 =
        MotorControl_countsToRpmTimes10(
            motor, status->encoderCounts);

    return true;
}

int32_t MotorControl_getLatestSpeedRpmTimes10(
    const MotorControl *motor)
{
    /* 不清除 statusReady，因此不会影响 UART0 电机状态查询。 */
    int32_t encoderCounts = motor->reportCounts;

    return MotorControl_countsToRpmTimes10(
        motor, encoderCounts);
}
