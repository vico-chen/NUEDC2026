#include "straight_task_control.h"

#define STRAIGHT_TASK_MOTOR_COUNT (4U)
#define PI_TIMES_1000 (3142U)

/* 使用可选日志回调报告直行任务状态变化。 */
static void StraightTaskControl_log(
    const StraightTaskControl *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

/* 安全取得编码器计数绝对值，忽略不同车轮安装方向的符号差异。 */
static uint32_t StraightTaskControl_absCount(int32_t value)
{
    if (value >= 0) {
        return (uint32_t) value;
    }
    return (uint32_t) (-(value + 1)) + 1U;
}

static uint32_t StraightTaskControl_distanceToAllWheelCounts(
    const StraightTaskControl *control, uint16_t distanceMm,
    uint16_t wheelDiameterMm)
{
    /*
     * 距离换算公式：
     * 目标计数 = 距离 × 四轮每圈计数总和 / (π × 轮径)。
     */
    uint32_t countsPerRevolutionSum = 0U;
    uint8_t i;
    uint64_t numerator;
    uint64_t denominator;

    if ((distanceMm == 0U) || (wheelDiameterMm == 0U)) {
        return 0U;
    }

    for (i = 0U; i < STRAIGHT_TASK_MOTOR_COUNT; i++) {
        const MotorControl_Config *motor = &control->config.motors[i]->config;
        countsPerRevolutionSum +=
            (uint32_t) motor->encoderPpr *
            (uint32_t) motor->gearRatio *
            (uint32_t) motor->encoderDecodeMultiplier;
    }

    numerator = (uint64_t) distanceMm *
        (uint64_t) countsPerRevolutionSum * 1000ULL;
    denominator = (uint64_t) PI_TIMES_1000 *
        (uint64_t) wheelDiameterMm;
    return (uint32_t) ((numerator + (denominator / 2ULL)) /
        denominator);
}

/* 累加四轮每个 10 ms 周期产生的编码器计数。 */
static void StraightTaskControl_accumulateEncoderCounts(
    StraightTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < STRAIGHT_TASK_MOTOR_COUNT; i++) {
        uint32_t count = StraightTaskControl_absCount(
            control->config.motors[i]->speedCountsPerSample);
        if (UINT32_MAX - control->encoderCount < count) {
            control->encoderCount = UINT32_MAX;
            return;
        }
        control->encoderCount += count;
    }
}

/* 四轮均进入零速死区后才确认停车完成。 */
static bool StraightTaskControl_carStopped(
    const StraightTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < STRAIGHT_TASK_MOTOR_COUNT; i++) {
        int32_t count =
            control->config.motors[i]->speedCountsPerSample;
        int32_t deadband =
            control->config.motors[i]->config.zeroSpeedDeadbandCounts;
        if ((count > deadband) || (count < -deadband)) {
            return false;
        }
    }
    return true;
}

void StraightTaskControl_init(StraightTaskControl *control,
    const StraightTask_Config *config)
{
    control->config = *config;
    StraightTaskControl_reset(control);
}

bool StraightTaskControl_start(StraightTaskControl *control,
    const StraightTask_Profile *profile)
{
    /* 距离、轮径或速度参数无效时禁止车辆启动。 */
    if ((profile == 0) || (profile->targetDistanceMm == 0U) ||
        (profile->wheelDiameterMm == 0U) ||
        (profile->cruiseRpm <= 0)) {
        return false;
    }

    control->profile = profile;
    control->state = STRAIGHT_TASK_RUNNING;
    control->accelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->encoderCount = 0U;
    /* 分别换算总距离和终点减速区间对应的编码器计数。 */
    control->targetEncoderCount =
        StraightTaskControl_distanceToAllWheelCounts(control,
            profile->targetDistanceMm, profile->wheelDiameterMm);
    control->decelerationEncoderCount =
        StraightTaskControl_distanceToAllWheelCounts(control,
            profile->decelerationDistanceMm,
            profile->wheelDiameterMm);
    CarControl_stop(control->config.car);
    StraightTaskControl_log(control,
        "STRAIGHT TASK STARTED 1500mm\r\n");
    return true;
}

void StraightTaskControl_reset(StraightTaskControl *control)
{
    /* 清除上一次里程并立即要求车辆停车。 */
    control->profile = 0;
    control->state = STRAIGHT_TASK_IDLE;
    control->accelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->encoderCount = 0U;
    control->targetEncoderCount = 0U;
    control->decelerationEncoderCount = 0U;
    CarControl_stop(control->config.car);
}

void StraightTaskControl_update(StraightTaskControl *control)
{
    uint32_t remainingCounts;
    int16_t speedRpm;

    switch (control->state) {
        /*
         * 行驶阶段包含三部分：
         * 起步按时间加速、中段匀速、最后按剩余距离连续减速。
         */
        case STRAIGHT_TASK_RUNNING:
            StraightTaskControl_accumulateEncoderCounts(control);
            if (control->encoderCount >= control->targetEncoderCount) {
                /* 达到 1.5 m 后目标速度归零，进入停稳确认。 */
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->state = STRAIGHT_TASK_BRAKE;
                StraightTaskControl_log(control,
                    "STRAIGHT DISTANCE REACHED, BRAKING\r\n");
                break;
            }

            remainingCounts =
                control->targetEncoderCount - control->encoderCount;
            if ((control->decelerationEncoderCount > 0U) &&
                (remainingCounts <=
                    control->decelerationEncoderCount)) {
                /*
                 * 已进入终点减速区：剩余距离越小，目标 RPM 越低。
                 * 到达终点前速度会逐渐接近 decelerationEndRpm。
                 */
                speedRpm = (int16_t) (
                    control->profile->decelerationEndRpm +
                    (((int32_t) (control->profile->cruiseRpm -
                        control->profile->decelerationEndRpm) *
                        remainingCounts) /
                        control->decelerationEncoderCount));
            } else {
                /* 尚未进入减速区，先完成起步斜坡再保持巡航速度。 */
                if (control->accelerationSamples <
                    control->profile->accelerationSamples) {
                    control->accelerationSamples++;
                }
                if (control->profile->accelerationSamples == 0U) {
                    speedRpm = control->profile->cruiseRpm;
                } else {
                    speedRpm = (int16_t) (((int32_t)
                        control->profile->cruiseRpm *
                        control->accelerationSamples) /
                        control->profile->accelerationSamples);
                }
            }
            CarControl_setMotion(control->config.car,
                CAR_CONTROL_FORWARD, speedRpm, 100U);
            break;

        /* 制动阶段等待真实轮速归零，并设置超时保护。 */
        case STRAIGHT_TASK_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    control->profile->brakeMinimumSamples) &&
                StraightTaskControl_carStopped(control)) {
                control->state = STRAIGHT_TASK_DONE;
                StraightTaskControl_log(control,
                    "STRAIGHT TASK DONE\r\n");
            } else if (control->brakeSamples >=
                control->profile->brakeTimeoutSamples) {
                CarControl_emergencyStop(control->config.car);
                control->state = STRAIGHT_TASK_FAULT;
                StraightTaskControl_log(control,
                    "STRAIGHT TASK BRAKE TIMEOUT\r\n");
            }
            break;

        case STRAIGHT_TASK_IDLE:
        case STRAIGHT_TASK_DONE:
        case STRAIGHT_TASK_FAULT:
        default:
            break;
    }
}

bool StraightTaskControl_isRunning(
    const StraightTaskControl *control)
{
    /* 行驶和制动阶段都视为模块正在占用车辆控制权。 */
    return (control->state == STRAIGHT_TASK_RUNNING) ||
           (control->state == STRAIGHT_TASK_BRAKE);
}

bool StraightTaskControl_isFinished(
    const StraightTaskControl *control)
{
    /* 正常完成或故障后，任务执行器都应停止计时。 */
    return (control->state == STRAIGHT_TASK_DONE) ||
           (control->state == STRAIGHT_TASK_FAULT);
}
