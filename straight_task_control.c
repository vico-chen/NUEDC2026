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
        (profile->cruiseRpm <= 0) ||
        (profile->decelerationEndRpm < 0) ||
        (profile->decelerationEndRpm > profile->cruiseRpm) ||
        (profile->decelerationDistanceMm >
            profile->targetDistanceMm)) {
        return false;
    }

    control->profile = profile;
    control->state = STRAIGHT_TASK_RUNNING;
    control->accelerationSamples = 0U;
    control->decelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->commandedRpm = 0;
    control->decelerationStartRpm = 0;
    control->decelerationStarted = false;
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
        "STRAIGHT TASK STARTED\r\n");
    return true;
}

void StraightTaskControl_reset(StraightTaskControl *control)
{
    /* 清除上一次里程并立即要求车辆停车。 */
    control->profile = 0;
    control->state = STRAIGHT_TASK_IDLE;
    control->accelerationSamples = 0U;
    control->decelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->commandedRpm = 0;
    control->decelerationStartRpm = 0;
    control->decelerationStarted = false;
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
         * 起步按时间加速、中段匀速、最后按配置时间连续减速。
         */
        case STRAIGHT_TASK_RUNNING:
            StraightTaskControl_accumulateEncoderCounts(control);

            /* 超过名义目标距离时钳位为 0，避免无符号减法下溢。 */
            if (control->encoderCount >= control->targetEncoderCount) {
                remainingCounts = 0U;
            } else {
                remainingCounts =
                    control->targetEncoderCount - control->encoderCount;
            }

            if (remainingCounts <=
                control->decelerationEncoderCount) {
                /*
                 * 第一次进入减速区时保存当前目标转速。后续每 10 ms
                 * 按 decelerationSamples 线性降低转速，因此目标速度
                 * 对时间呈直线变化，而不再随剩余距离计算。
                 */
                if (!control->decelerationStarted) {
                    control->decelerationStarted = true;
                    control->decelerationSamples = 0U;
                    control->decelerationStartRpm =
                        control->commandedRpm;
                    if (control->decelerationStartRpm <
                        control->profile->decelerationEndRpm) {
                        control->decelerationStartRpm =
                            control->profile->decelerationEndRpm;
                    }
                }

                /*
                 * 减速时间为 0 时不执行减速斜坡，进入减速区后立即制动。
                 */
                if (control->profile->decelerationSamples == 0U) {
                    CarControl_stop(control->config.car);
                    control->commandedRpm = 0;
                    control->brakeSamples = 0U;
                    control->state = STRAIGHT_TASK_BRAKE;
                    StraightTaskControl_log(control,
                        "STRAIGHT DECELERATION DONE, BRAKING\r\n");
                    break;
                }

                if (control->decelerationSamples <
                    control->profile->decelerationSamples) {
                    control->decelerationSamples++;
                }

                /*
                 * 减速计时一结束就停车，不再等待编码器到达目标距离。
                 */
                if (control->decelerationSamples >=
                    control->profile->decelerationSamples) {
                    CarControl_stop(control->config.car);
                    control->commandedRpm = 0;
                    control->brakeSamples = 0U;
                    control->state = STRAIGHT_TASK_BRAKE;
                    StraightTaskControl_log(control,
                        "STRAIGHT DECELERATION DONE, BRAKING\r\n");
                    break;
                }

                speedRpm = (int16_t) (
                    control->decelerationStartRpm -
                    (((int32_t) (control->decelerationStartRpm -
                        control->profile->decelerationEndRpm) *
                        control->decelerationSamples) /
                        control->profile->decelerationSamples));
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
            control->commandedRpm = speedRpm;
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
