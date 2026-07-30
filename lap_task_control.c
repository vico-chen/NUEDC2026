#include "lap_task_control.h"

#define LAP_MOTOR_COUNT (4U)
#define PI_TIMES_1000   (3142U)

static void LapTask_log(const LapTaskControl *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

static uint32_t LapTask_absCount(int32_t value)
{
    return (value >= 0) ? (uint32_t) value
                        : (uint32_t) (-(value + 1)) + 1U;
}

static uint32_t LapTask_distanceCounts(
    const LapTaskControl *control, uint16_t distanceMm,
    uint16_t wheelDiameterMm)
{
    uint32_t countsPerRevolutionSum = 0U;
    uint64_t numerator;
    uint64_t denominator;
    uint8_t i;

    if ((distanceMm == 0U) || (wheelDiameterMm == 0U)) {
        return 0U;
    }
    for (i = 0U; i < LAP_MOTOR_COUNT; i++) {
        const MotorControl_Config *motor =
            &control->config.motors[i]->config;
        countsPerRevolutionSum +=
            (uint32_t) motor->encoderPpr * motor->gearRatio *
            motor->encoderDecodeMultiplier;
    }
    numerator = (uint64_t) distanceMm *
        countsPerRevolutionSum * 1000ULL;
    denominator = (uint64_t) PI_TIMES_1000 * wheelDiameterMm;
    return (uint32_t) ((numerator + denominator / 2ULL) / denominator);
}

static void LapTask_accumulateCounts(LapTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < LAP_MOTOR_COUNT; i++) {
        uint32_t count = LapTask_absCount(
            control->config.motors[i]->speedCountsPerSample);
        if (UINT32_MAX - control->finishEncoderCount < count) {
            control->finishEncoderCount = UINT32_MAX;
            return;
        }
        control->finishEncoderCount += count;
    }
}

static bool LapTask_carStopped(const LapTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < LAP_MOTOR_COUNT; i++) {
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

static void LapTask_applyAcceleration(LapTaskControl *control)
{
    int16_t cruiseRpm = control->profile->cruiseRpm;
    int16_t straightRpm = control->profile->straightRpm;
    int16_t speedRpm;

    if (control->accelerationSamples <
        control->profile->accelerationSamples) {
        control->accelerationSamples++;
    }
    speedRpm = (control->profile->accelerationSamples == 0U) ?
        cruiseRpm :
        (int16_t) (((int32_t) cruiseRpm *
            control->accelerationSamples) /
            control->profile->accelerationSamples);

    if (straightRpm < cruiseRpm) {
        straightRpm = cruiseRpm;
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
                    CAR_CONTROL_FORWARD) ? straightRpm : cruiseRpm;
        } else {
            speedRpm = (int16_t) (cruiseRpm +
                (((int32_t) (straightRpm - cruiseRpm) *
                    control->straightAccelerationSamples) /
                    control->profile->straightAccelerationSamples));
        }
    } else {
        control->straightAccelerationSamples = 0U;
    }
    LineTracking_setSpeed(control->config.lineTracking, speedRpm);
}

static bool LapTask_intersectionDetected(LapTaskControl *control)
{
    uint8_t activeCount =
        LineTracking_getActiveCount(control->config.lineTracking);

    if (activeCount >= GRAYSCALE_SENSOR_CHANNELS) {
        control->intersectionConfirmCount = 0U;
        return true;
    }
    if (activeCount >=
        control->profile->intersectionActiveThreshold) {
        if (control->intersectionConfirmCount < 255U) {
            control->intersectionConfirmCount++;
        }
        return control->intersectionConfirmCount >=
            control->profile->intersectionConfirmSamples;
    }
    control->intersectionConfirmCount = 0U;
    return false;
}

void LapTaskControl_init(LapTaskControl *control,
    const LapTask_Config *config)
{
    *control = (LapTaskControl) {0};
    control->config = *config;
    LapTaskControl_reset(control);
}

bool LapTaskControl_start(LapTaskControl *control, uint8_t taskNumber,
    const LapTask_Profile *profile)
{
    if ((profile == 0) || (profile->cruiseRpm <= 0) ||
        (profile->wheelDiameterMm == 0U)) {
        return false;
    }

    control->profile = profile;
    control->state = LAP_TASK_FOLLOW;
    control->taskNumber = taskNumber;
    control->intersectionConfirmCount = 0U;
    control->accelerationSamples = 0U;
    control->straightAccelerationSamples = 0U;
    control->decelerationSamples = 0U;
    control->brakeSamples = 0U;
    control->finishStartRpm = 0;
    control->finishMotion = CAR_CONTROL_FORWARD;
    control->finishInnerPercent = 100U;
    control->finishEncoderCount = 0U;
    control->finishTargetEncoderCount = LapTask_distanceCounts(control,
        profile->finishAdvanceMm, profile->wheelDiameterMm);

    LineTracking_setSpeed(control->config.lineTracking, 0);
    LineTracking_setDebugEnabled(control->config.lineTracking, false);
    LineTracking_reset(control->config.lineTracking);
    LineTracking_setEnabled(control->config.lineTracking, true);
    LapTask_log(control, "LAP_TASK_STARTED\r\n");
    return true;
}

void LapTaskControl_reset(LapTaskControl *control)
{
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
    int16_t endRpm;

    switch (control->state) {
        case LAP_TASK_FOLLOW:
            LapTask_applyAcceleration(control);
            /*
             * 每个周期只读取一次灰度：巡线先更新并保存滤波结果，
             * 路口检测直接复用该结果，避免两次读取之间数据不一致。
             */
            LineTracking_update(control->config.lineTracking);
            if (LapTask_intersectionDetected(control)) {
                control->finishStartRpm =
                    LineTracking_getSpeed(control->config.lineTracking);
                control->finishMotion =
                    CarControl_getMotion(control->config.car);
                control->finishInnerPercent =
                    CarControl_getTurnInnerPercent(control->config.car);
                if ((control->finishMotion != CAR_CONTROL_FORWARD) &&
                    (control->finishMotion != CAR_CONTROL_FORWARD_LEFT) &&
                    (control->finishMotion != CAR_CONTROL_FORWARD_RIGHT)) {
                    control->finishMotion = CAR_CONTROL_FORWARD;
                    control->finishInnerPercent = 100U;
                }
                control->finishEncoderCount = 0U;
                control->state = LAP_TASK_FINISH_ADVANCE;
                LineTracking_setEnabled(
                    control->config.lineTracking, false);
                LineTracking_reset(control->config.lineTracking);
                CarControl_setMotion(control->config.car,
                    control->finishMotion, control->finishStartRpm,
                    control->finishInnerPercent);
                LapTask_log(control,
                    "LAP_INTERSECTION_ADVANCING\r\n");
            }
            break;

        case LAP_TASK_FINISH_ADVANCE:
            LapTask_accumulateCounts(control);
            CarControl_setMotion(control->config.car,
                control->finishMotion, control->finishStartRpm,
                control->finishInnerPercent);
            if (control->finishEncoderCount >=
                control->finishTargetEncoderCount) {
                control->decelerationSamples = 0U;
                control->state = LAP_TASK_DECELERATE;
                LapTask_log(control, "LAP_DECELERATING\r\n");
            }
            break;

        case LAP_TASK_DECELERATE:
            endRpm = control->profile->decelerationEndRpm;
            if (endRpm > control->finishStartRpm) {
                endRpm = control->finishStartRpm;
            }
            if (control->decelerationSamples <
                control->profile->decelerationSamples) {
                control->decelerationSamples++;
            }
            speedRpm = (control->profile->decelerationSamples == 0U) ?
                endRpm :
                (int16_t) (control->finishStartRpm -
                    (((int32_t) (control->finishStartRpm - endRpm) *
                        control->decelerationSamples) /
                        control->profile->decelerationSamples));
            CarControl_setMotion(control->config.car,
                control->finishMotion, speedRpm,
                control->finishInnerPercent);
            if (control->decelerationSamples >=
                control->profile->decelerationSamples) {
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->state = LAP_TASK_BRAKE;
                LapTask_log(control, "LAP_BRAKING\r\n");
            }
            break;

        case LAP_TASK_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    control->profile->brakeMinimumSamples) &&
                LapTask_carStopped(control)) {
                control->state = LAP_TASK_DONE;
                LapTask_log(control, "LAP_TASK_DONE\r\n");
            } else if (control->brakeSamples >=
                control->profile->brakeTimeoutSamples) {
                CarControl_emergencyStop(control->config.car);
                control->state = LAP_TASK_FAULT;
                LapTask_log(control, "LAP_BRAKE_TIMEOUT\r\n");
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
    return (control->state >= LAP_TASK_FOLLOW) &&
           (control->state <= LAP_TASK_BRAKE);
}

bool LapTaskControl_isFinished(const LapTaskControl *control)
{
    return (control->state == LAP_TASK_DONE) ||
           (control->state == LAP_TASK_FAULT);
}
