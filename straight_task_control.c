#include "straight_task_control.h"

#define STRAIGHT_MOTOR_COUNT (4U)
#define PI_TIMES_1000        (3142U)

static void StraightTask_log(
    const StraightTaskControl *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

static uint32_t StraightTask_absCount(int32_t value)
{
    return (value >= 0) ? (uint32_t) value
                        : (uint32_t) (-(value + 1)) + 1U;
}

static uint32_t StraightTask_distanceCounts(
    const StraightTaskControl *control, uint16_t distanceMm,
    uint16_t wheelDiameterMm)
{
    uint32_t countsPerRevolutionSum = 0U;
    uint64_t numerator;
    uint64_t denominator;
    uint8_t i;

    if ((distanceMm == 0U) || (wheelDiameterMm == 0U)) {
        return 0U;
    }
    for (i = 0U; i < STRAIGHT_MOTOR_COUNT; i++) {
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

static void StraightTask_accumulateCounts(StraightTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < STRAIGHT_MOTOR_COUNT; i++) {
        uint32_t count = StraightTask_absCount(
            control->config.motors[i]->speedCountsPerSample);
        if (UINT32_MAX - control->encoderCount < count) {
            control->encoderCount = UINT32_MAX;
            return;
        }
        control->encoderCount += count;
    }
}

static bool StraightTask_carStopped(
    const StraightTaskControl *control)
{
    uint8_t i;

    for (i = 0U; i < STRAIGHT_MOTOR_COUNT; i++) {
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
    *control = (StraightTaskControl) {0};
    control->config = *config;
    StraightTaskControl_reset(control);
}

bool StraightTaskControl_start(StraightTaskControl *control,
    const StraightTask_Profile *profile)
{
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
    control->targetEncoderCount = StraightTask_distanceCounts(control,
        profile->targetDistanceMm, profile->wheelDiameterMm);
    control->decelerationEncoderCount =
        StraightTask_distanceCounts(control,
            profile->decelerationDistanceMm,
            profile->wheelDiameterMm);
    CarControl_stop(control->config.car);
    StraightTask_log(control, "STRAIGHT_TASK_STARTED_2000MM\r\n");
    return true;
}

void StraightTaskControl_reset(StraightTaskControl *control)
{
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
        case STRAIGHT_TASK_RUNNING:
            StraightTask_accumulateCounts(control);
            if (control->encoderCount >= control->targetEncoderCount) {
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->state = STRAIGHT_TASK_BRAKE;
                StraightTask_log(control,
                    "STRAIGHT_DISTANCE_REACHED\r\n");
                break;
            }

            remainingCounts =
                control->targetEncoderCount - control->encoderCount;
            if ((control->decelerationEncoderCount > 0U) &&
                (remainingCounts <=
                    control->decelerationEncoderCount)) {
                speedRpm = (int16_t) (
                    control->profile->decelerationEndRpm +
                    (((int32_t) (control->profile->cruiseRpm -
                        control->profile->decelerationEndRpm) *
                        remainingCounts) /
                        control->decelerationEncoderCount));
            } else {
                if (control->accelerationSamples <
                    control->profile->accelerationSamples) {
                    control->accelerationSamples++;
                }
                speedRpm =
                    (control->profile->accelerationSamples == 0U) ?
                    control->profile->cruiseRpm :
                    (int16_t) (((int32_t)
                        control->profile->cruiseRpm *
                        control->accelerationSamples) /
                        control->profile->accelerationSamples);
            }
            CarControl_setMotion(control->config.car,
                CAR_CONTROL_FORWARD, speedRpm, 100U);
            break;

        case STRAIGHT_TASK_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    control->profile->brakeMinimumSamples) &&
                StraightTask_carStopped(control)) {
                control->state = STRAIGHT_TASK_DONE;
                StraightTask_log(control, "STRAIGHT_TASK_DONE\r\n");
            } else if (control->brakeSamples >=
                control->profile->brakeTimeoutSamples) {
                CarControl_emergencyStop(control->config.car);
                control->state = STRAIGHT_TASK_FAULT;
                StraightTask_log(control,
                    "STRAIGHT_BRAKE_TIMEOUT\r\n");
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
    return (control->state == STRAIGHT_TASK_RUNNING) ||
           (control->state == STRAIGHT_TASK_BRAKE);
}

bool StraightTaskControl_isFinished(
    const StraightTaskControl *control)
{
    return (control->state == STRAIGHT_TASK_DONE) ||
           (control->state == STRAIGHT_TASK_FAULT);
}
