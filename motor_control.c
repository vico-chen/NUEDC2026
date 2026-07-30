#include "motor_control.h"

/* 计算减速箱输出轴旋转一圈对应的编码器计数。 */
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

/* 避免引入数学库，仅提供控制算法需要的浮点绝对值。 */
static float MotorControl_absFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 将浮点 PID 输出按四舍五入转换为带符号百分比。 */
static int16_t MotorControl_roundOutput(float output)
{
    return (output >= 0.0f) ? (int16_t) (output + 0.5f)
                            : (int16_t) (output - 0.5f);
}

/*
 * 把带符号 PWM 百分比转换为 TB6612 方向电平和比较值。
 * 本工程 PWM 为低有效比较逻辑，所以占空比越大，比较值越小。
 */
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
        /* PWM=0 时撤销驱动，IN1/IN2 同时拉低。 */
        DL_GPIO_clearPins(motor->config.directionIn1Port,
            motor->config.directionIn1Pin);
        DL_GPIO_clearPins(motor->config.directionIn2Port,
            motor->config.directionIn2Pin);
        compareValue = loadValue;
    } else {
        if (pwmPercent > 0) {
            /* 正输出：IN1=1、IN2=0。 */
            DL_GPIO_setPins(motor->config.directionIn1Port,
                motor->config.directionIn1Pin);
            DL_GPIO_clearPins(motor->config.directionIn2Port,
                motor->config.directionIn2Pin);
            pwmMagnitude = (uint16_t) pwmPercent;
        } else {
            /* 负输出：IN1=0、IN2=1。 */
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

/*
 * 有符号增量式 PID：
 *   Δu(k) = Kp[e(k)-e(k-1)] + Ki·e(k)
 *         + Kd[e(k)-2e(k-1)+e(k-2)]
 *
 * 与只看速度绝对值的传统写法不同，这里允许在内轮被车身拖快时输出
 * 有限的反向力矩，从而在小半径弧线中真正约束内轮转速。
 */
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
        /* 滑行模式完全撤销控制量，不执行主动制动。 */
        MotorControl_resetPid(motor);
        motor->pidDirection = 0;
        return 0;
    }

    if (targetRpm != 0) {
        /* 目标方向变化时清空历史项，避免旧积分造成反冲。 */
        requestedDirection = (targetRpm > 0) ? 1 : -1;
        if (requestedDirection != motor->pidDirection) {
            MotorControl_resetPid(motor);
            motor->pidDirection = requestedDirection;
        }
    } else if (MotorControl_absFloat(measuredCountsPerSample) <=
               (float) motor->config.zeroSpeedDeadbandCounts) {
        /* 进入零速死区后停止制动，防止残余积分把静止车轮反向拉动。 */
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
     * 按目标方向设置不对称限幅：行驶中只允许较小的反向制动，
     * 目标为零时才使用停车制动力，避免弯道内轮被突然反拖。
     */
    if (targetRpm > 0) {
        outputMinimum = -motor->config.runningBrakeMaxPercent;
        outputMaximum = motor->config.outputMaxPercent;
    } else if (targetRpm < 0) {
        outputMinimum = -motor->config.outputMaxPercent;
        outputMaximum = motor->config.runningBrakeMaxPercent;
    } else {
        outputMinimum = -motor->config.stopBrakeMaxPercent;
        outputMaximum = motor->config.stopBrakeMaxPercent;
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

/* 设置目标转速并限幅；任何新目标都会退出滑行模式。 */
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

/* 紧急滑行：立即清除输出和 PID，不等待下一次定时中断。 */
void MotorControl_coast(MotorControl *motor)
{
    motor->targetRpm = 0;
    motor->coastMode = true;
    motor->pidDirection = 0;
    MotorControl_resetPid(motor);
    MotorControl_applyOutput(motor, 0);
}

/*
 * B 相每次翻转时读取 A/B 当前电平，通过两相组合判断旋转方向。
 * 该方法属于二倍频解码。
 */
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

/* 10 ms 速度环入口。编码器原始增量仍保留给 100 ms 状态统计。 */
void MotorControl_update(MotorControl *motor)
{
    float alpha = motor->config.speedFilterAlpha;

    motor->speedCountsPerSample = motor->encoderCount;
    motor->encoderCount = 0;

    if ((alpha <= 0.0f) || (alpha > 1.0f)) {
        /* 配置异常时退化为不滤波，避免控制器失效。 */
        alpha = 1.0f;
    }
    if (!motor->speedFilterReady) {
        /* 首帧直接赋值，避免从 0 缓慢爬升造成启动滞后。 */
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

    /* 将 reportSamples 个周期的总计数换算为 0.1 RPM 单位。 */
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
