#include "lap_task_control.h"

#define LAP_TASK_MOTOR_COUNT (4U)
#define PI_TIMES_1000 (3142U)

/* 使用日志回调报告状态变化；未配置日志时不影响任务运行。 */
static void LapTaskControl_log(
    const LapTaskControl *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

/* 安全取得有符号编码器计数的绝对值，兼容 INT32_MIN。 */
static uint32_t LapTaskControl_absCount(int32_t value)
{
    if (value >= 0) {
        return (uint32_t) value;
    }
    return (uint32_t) (-(value + 1)) + 1U;
}

/*
 * 将毫米距离换算为四个车轮编码器计数总和：
 * 距离 / 轮胎周长 × 每圈编码器计数。
 * 使用四轮计数总和，可以减少单个编码器偶发丢计数带来的影响。
 */
static uint32_t LapTaskControl_distanceToAllWheelCounts(
    const LapTaskControl *control, uint16_t distanceMm,
    uint16_t wheelDiameterMm)
{
    uint32_t countsPerRevolutionSum = 0U;
    uint8_t i;
    uint64_t numerator;
    uint64_t denominator;

    if ((distanceMm == 0U) || (wheelDiameterMm == 0U)) {
        return 0U;
    }

    for (i = 0U; i < LAP_TASK_MOTOR_COUNT; i++) {
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

/* 每个 10 ms 周期把四轮本周期计数累加到终点前进距离。 */
static void LapTaskControl_accumulateEncoderCounts(
    LapTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < LAP_TASK_MOTOR_COUNT; i++) {
        uint32_t count = LapTaskControl_absCount(
            control->config.motors[i]->speedCountsPerSample);
        if (UINT32_MAX - control->finishEncoderCount < count) {
            control->finishEncoderCount = UINT32_MAX;
            return;
        }
        control->finishEncoderCount += count;
    }
}

/* 四个车轮都进入各自的零速死区后，才认为车辆真正停稳。 */
static bool LapTaskControl_carStopped(const LapTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < LAP_TASK_MOTOR_COUNT; i++) {
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

/* 按线性斜坡把巡线速度从 0 提升至配置的巡航 RPM。 */
static void LapTaskControl_applyAcceleration(LapTaskControl *control)
{
    int16_t straightRpm;
    int16_t speedRpm;

    /* 首先按原有斜坡从 0 加速到弯道/普通巡线速度。 */
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

    /*
     * 完成基础加速后，根据上一周期的巡线动作判断是否处于直行。
     * 直行时逐步提升到 straightRpm；出现转弯修正时逐步退回 cruiseRpm，
     * 避免数字传感器短暂跳变造成速度目标瞬间切换。
     */
    straightRpm = control->profile->straightRpm;
    if (straightRpm < control->profile->cruiseRpm) {
        straightRpm = control->profile->cruiseRpm;
    }

    if (control->accelerationSamples >=
        control->profile->accelerationSamples) {
        if (CarControl_getMotion(control->config.car) ==
            CAR_CONTROL_FORWARD) {
            if (control->straightAccelerationSamples <
                control->profile->straightAccelerationSamples) {
                control->straightAccelerationSamples++;
            }
        } else if (control->straightAccelerationSamples > 0U) {
            control->straightAccelerationSamples--;
        }

        if (control->profile->straightAccelerationSamples == 0U) {
            speedRpm =
                (CarControl_getMotion(control->config.car) ==
                    CAR_CONTROL_FORWARD) ?
                straightRpm : control->profile->cruiseRpm;
        } else {
            speedRpm = (int16_t) (
                control->profile->cruiseRpm +
                (((int32_t) (straightRpm -
                    control->profile->cruiseRpm) *
                    control->straightAccelerationSamples) /
                    control->profile->straightAccelerationSamples));
        }
    } else {
        control->straightAccelerationSamples = 0U;
    }

    LineTracking_setSpeed(control->config.lineTracking, speedRpm);
}

/*
 * 八路全部有效时立即确认十字路口；
 * 六路以上有效时必须连续达到确认次数，降低误判概率。
 */
static bool LapTaskControl_detectIntersection(
    LapTaskControl *control)
{
    uint8_t activeCount = LineTracking_readActiveCount();

    if (activeCount >= GRAYSCALE_SENSOR_CHANNELS) {
        control->intersectionConfirmCount = 0U;
        return true;
    }
    if (activeCount >=
        control->profile->intersectionActiveThreshold) {
        if (control->intersectionConfirmCount < 255U) {
            control->intersectionConfirmCount++;
        }
        return (control->intersectionConfirmCount >=
            control->profile->intersectionConfirmSamples);
    }
    control->intersectionConfirmCount = 0U;
    return false;
}

void LapTaskControl_init(LapTaskControl *control,
    const LapTask_Config *config)
{
    control->config = *config;
    LapTaskControl_reset(control);
}

bool LapTaskControl_start(LapTaskControl *control, uint8_t taskNumber,
    const LapTask_Profile *profile)
{
    /* 参数不完整时拒绝启动，车辆保持停止状态。 */
    if ((profile == 0) || (profile->cruiseRpm <= 0) ||
        (profile->wheelDiameterMm == 0U)) {
        return false;
    }

    control->profile = profile;
    control->taskNumber = taskNumber;
    control->state = LAP_TASK_FOLLOW;
    control->intersectionConfirmCount = 0U;
    control->accelerationSamples = 0U;
    control->straightAccelerationSamples = 0U;
    control->decelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->finishStartRpm = 0;
    control->finishMotion = CAR_CONTROL_FORWARD;
    control->finishInnerPercent = 100U;
    control->finishEncoderCount = 0U;
    /* 根据轮径和编码器规格预先计算终点后继续前进的目标计数。 */
    control->finishTargetEncoderCount =
        LapTaskControl_distanceToAllWheelCounts(control,
            profile->finishAdvanceMm, profile->wheelDiameterMm);

    LineTracking_setSpeed(control->config.lineTracking, 0);
    LineTracking_setDebugEnabled(control->config.lineTracking, false);
    LineTracking_reset(control->config.lineTracking);
    LineTracking_setEnabled(control->config.lineTracking, true);
    LapTaskControl_log(control, "LAP TASK STARTED\r\n");
    return true;
}

void LapTaskControl_reset(LapTaskControl *control)
{
    /* 清空状态机，并确保巡线模块和车辆都停止输出。 */
    control->profile = 0;
    control->state = LAP_TASK_IDLE;
    control->taskNumber = 0U;
    control->intersectionConfirmCount = 0U;
    control->accelerationSamples = 0U;
    control->straightAccelerationSamples = 0U;
    control->decelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->finishStartRpm = 0;
    control->finishMotion = CAR_CONTROL_FORWARD;
    control->finishInnerPercent = 100U;
    control->finishEncoderCount = 0U;
    control->finishTargetEncoderCount = 0U;
    LineTracking_setEnabled(control->config.lineTracking, false);
    LineTracking_setDebugEnabled(control->config.lineTracking, false);
    LineTracking_reset(control->config.lineTracking);
    CarControl_stop(control->config.car);
}

void LapTaskControl_update(LapTaskControl *control)
{
    int16_t speedRpm;
    int16_t decelerationEndRpm;

    switch (control->state) {
        /* 阶段1：平滑加速巡线，持续寻找终点十字路口。 */
        case LAP_TASK_FOLLOW:
            LapTaskControl_applyAcceleration(control);
            if (LapTaskControl_detectIntersection(control)) {
                /*
                 * 保存识别终点时加速斜坡实际下发的速度。
                 * 不能改用配置巡航速度，否则尚未加速完成时会先突增速度再减速。
                 */
                control->finishStartRpm =
                    LineTracking_getSpeed(control->config.lineTracking);
                control->finishMotion =
                    CarControl_getMotion(control->config.car);
                control->finishInnerPercent =
                    CarControl_getTurnInnerPercent(control->config.car);

                /*
                 * 一圈模组的巡线只应产生前进、前进左转或前进右转。
                 * 若状态异常则退回等速直行，避免把倒车或原地转向锁到终点阶段。
                 */
                if ((control->finishMotion != CAR_CONTROL_FORWARD) &&
                    (control->finishMotion != CAR_CONTROL_FORWARD_LEFT) &&
                    (control->finishMotion != CAR_CONTROL_FORWARD_RIGHT)) {
                    control->finishMotion = CAR_CONTROL_FORWARD;
                    control->finishInnerPercent = 100U;
                }
                control->finishEncoderCount = 0U;
                control->state = LAP_TASK_FINISH_ADVANCE;

                /*
                 * 终点十字线会让多个灰度通道同时跳变，不能再交给巡线 PID，
                 * 否则左右轮可能在经过横线时瞬间切换修正方向并产生抖动。
                 * 确认终点后立即关闭巡线、清空全部滤波和 PID 历史，
                 * 再明确下发等速直行，避免沿用上一周期的转弯目标。
                 */
                LineTracking_setEnabled(
                    control->config.lineTracking, false);
                LineTracking_reset(control->config.lineTracking);
                CarControl_setMotion(control->config.car,
                    control->finishMotion, control->finishStartRpm,
                    control->finishInnerPercent);
                LapTaskControl_log(control,
                    "LAP FINISH INTERSECTION, ADVANCING\r\n");
            } else {
                LineTracking_update(control->config.lineTracking);
            }
            break;

        /*
         * 阶段2：已经扫到终点，但暂不停车。
         * 保持巡线和巡航速度，并按编码器再前进 finishAdvanceMm。
         */
        case LAP_TASK_FINISH_ADVANCE:
            LapTaskControl_accumulateEncoderCounts(control);
            CarControl_setMotion(control->config.car,
                control->finishMotion, control->finishStartRpm,
                control->finishInnerPercent);
            if (control->finishEncoderCount >=
                control->finishTargetEncoderCount) {
                control->decelerationSamples = 0U;
                control->state = LAP_TASK_DECELERATE;
                LapTaskControl_log(control,
                    "LAP ADVANCE DONE, DECELERATING\r\n");
            }
            break;

        /*
         * 阶段3：继续巡线，同时在指定周期内把速度线性降低到
         * decelerationEndRpm，避免终点处急停。
         */
        case LAP_TASK_DECELERATE:
            /*
             * 结束速度不能高于终点入口速度，避免低速经过终点时，
             * “减速”阶段反而把车辆重新加速到配置结束速度。
             */
            decelerationEndRpm =
                control->profile->decelerationEndRpm;
            if (decelerationEndRpm > control->finishStartRpm) {
                decelerationEndRpm = control->finishStartRpm;
            }

            if (control->decelerationSamples <
                control->profile->decelerationSamples) {
                control->decelerationSamples++;
            }

            if (control->profile->decelerationSamples == 0U) {
                speedRpm = decelerationEndRpm;
            } else {
                speedRpm = (int16_t) (
                    control->finishStartRpm -
                    (((int32_t) (control->finishStartRpm -
                        decelerationEndRpm) *
                        control->decelerationSamples) /
                        control->profile->decelerationSamples));
            }
            CarControl_setMotion(control->config.car,
                control->finishMotion, speedRpm,
                control->finishInnerPercent);

            if (control->decelerationSamples >=
                control->profile->decelerationSamples) {
                LineTracking_setEnabled(
                    control->config.lineTracking, false);
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->state = LAP_TASK_BRAKE;
                LapTaskControl_log(control, "LAP BRAKING\r\n");
            }
            break;

        /* 阶段4：目标速度归零，等待四轮实际速度进入死区。 */
        case LAP_TASK_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    control->profile->brakeMinimumSamples) &&
                LapTaskControl_carStopped(control)) {
                control->state = LAP_TASK_DONE;
                LapTaskControl_log(control, "LAP TASK DONE\r\n");
            } else if (control->brakeSamples >=
                control->profile->brakeTimeoutSamples) {
                CarControl_emergencyStop(control->config.car);
                control->state = LAP_TASK_FAULT;
                LapTaskControl_log(control,
                    "LAP TASK BRAKE TIMEOUT\r\n");
            }
            break;

        case LAP_TASK_IDLE:
        case LAP_TASK_DONE:
        case LAP_TASK_FAULT:
        default:
            break;
    }
}

bool LapTaskControl_isRunning(const LapTaskControl *control)
{
    /* FOLLOW 到 BRAKE 都属于任务正在控制车辆。 */
    return (control->state >= LAP_TASK_FOLLOW) &&
           (control->state <= LAP_TASK_BRAKE);
}

bool LapTaskControl_isFinished(const LapTaskControl *control)
{
    /* DONE 和 FAULT 都表示本轮状态机已经停止推进。 */
    return (control->state == LAP_TASK_DONE) ||
           (control->state == LAP_TASK_FAULT);
}
