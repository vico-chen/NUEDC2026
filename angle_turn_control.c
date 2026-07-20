#include "angle_turn_control.h"
#include "mpu6050_angle.h"

static float AngleTurnControl_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float AngleTurnControl_getNormalizedYaw(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZDegrees() *
           (float) control->config.leftTurnYawSign;
}

static float AngleTurnControl_getNormalizedRate(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZRateDps() *
           (float) control->config.leftTurnYawSign;
}

static void AngleTurnControl_enterHold(AngleTurnControl *control)
{
    CarControl_stop(control->config.car);
    control->holding = true;
}

void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config)
{
    control->config = *config;
    control->targetYawDegrees = 0.0f;
    control->maximumRpm = config->defaultMaximumRpm;
    control->settledCount = 0U;
    control->elapsedSamples = 0U;
    control->active = false;
    control->holding = false;
}

bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t maximumRpm)
{
    float direction;

    if (relativeAngleDegrees <= 0.0f) {
        return false;
    }
    if (maximumRpm <= 0) {
        maximumRpm = control->config.defaultMaximumRpm;
    }
    if (maximumRpm < control->config.minimumRpm) {
        maximumRpm = control->config.minimumRpm;
    }
    if (maximumRpm > control->config.car->config.maximumSpeedRpm) {
        maximumRpm = control->config.car->config.maximumSpeedRpm;
    }

    /* Each command is a relative turn, so use its start direction as 0 deg. */
    CarControl_stop(control->config.car);
    MPU6050_Angle_reset();
    direction = turnLeft ? 1.0f : -1.0f;
    control->targetYawDegrees = direction * relativeAngleDegrees;
    control->maximumRpm = maximumRpm;
    control->settledCount = 0U;
    control->elapsedSamples = 0U;
    control->holding = false;
    control->active = true;
    return true;
}

AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control)
{
    float error;
    float yawRate;
    float errorAbs;
    float rateAbs;
    float commandRpm;
    float commandMagnitude;
    int16_t speedRpm;

    if (!control->active) {
        return ANGLE_TURN_RESULT_NONE;
    }

    control->elapsedSamples++;
    if (control->elapsedSamples >= control->config.timeoutSamples) {
        CarControl_stop(control->config.car);
        control->active = false;
        control->holding = false;
        return ANGLE_TURN_RESULT_TIMEOUT;
    }

    error = AngleTurnControl_getErrorDegrees(control);
    yawRate = AngleTurnControl_getNormalizedRate(control);
    errorAbs = AngleTurnControl_abs(error);
    rateAbs = AngleTurnControl_abs(yawRate);

    /*
     * Hold / settle stage: keep braking once the target window is reached.
     * Re-drive only if the error grows past the hysteresis band. This stops
     * the classic min-RPM kick / overshoot / kick-again chatter.
     */
    if (control->holding) {
        CarControl_stop(control->config.car);

        if (errorAbs > control->config.holdReleaseDegrees) {
            control->holding = false;
            control->settledCount = 0U;
        } else {
            if ((errorAbs <= control->config.angleToleranceDegrees) &&
                (rateAbs <= control->config.stoppedRateToleranceDps)) {
                control->settledCount++;
                if (control->settledCount >=
                    control->config.settleSamples) {
                    control->active = false;
                    control->holding = false;
                    return ANGLE_TURN_RESULT_COMPLETED;
                }
            } else if (rateAbs >
                       (control->config.stoppedRateToleranceDps * 2.0f)) {
                control->settledCount = 0U;
            }
            return ANGLE_TURN_RESULT_NONE;
        }
    }

    if (errorAbs <= control->config.angleToleranceDegrees) {
        AngleTurnControl_enterHold(control);
        if (rateAbs <= control->config.stoppedRateToleranceDps) {
            control->settledCount = 1U;
        } else {
            control->settledCount = 0U;
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    /*
     * Outer-loop PD controller:
     *   angle error requests rotation; measured yaw rate anticipates inertia.
     * If damping asks for the opposite direction before reaching the target,
     * actively brake and hold instead of reversing the wheels.
     */
    commandRpm =
        control->config.kpRpmPerDegree * error -
        control->config.kdRpmPerDps * yawRate;
    if ((commandRpm * error) <= 0.0f) {
        CarControl_stop(control->config.car);
        if (errorAbs <= control->config.holdReleaseDegrees) {
            control->holding = true;
            control->settledCount = 0U;
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    commandMagnitude = AngleTurnControl_abs(commandRpm);
    if (commandMagnitude < (float) control->config.minimumRpm) {
        if (errorAbs >= control->config.minRpmEngageDegrees) {
            commandMagnitude = (float) control->config.minimumRpm;
        } else if (commandMagnitude < 10.0f) {
            /*
             * Near the target, a forced minimumRpm kick is what creates the
             * visible shake. Soft commands below ~10 RPM are not useful, so
             * brake; only latch into hold inside the hysteresis window.
             */
            CarControl_stop(control->config.car);
            if (errorAbs <= control->config.holdReleaseDegrees) {
                control->holding = true;
                control->settledCount = 0U;
            }
            return ANGLE_TURN_RESULT_NONE;
        }
        /* else: keep soft proportional RPM without boosting to minimumRpm */
    }
    if (commandMagnitude > (float) control->maximumRpm) {
        commandMagnitude = (float) control->maximumRpm;
    }
    speedRpm = (int16_t) (commandMagnitude + 0.5f);

    CarControl_setMotion(control->config.car,
        (commandRpm > 0.0f) ? CAR_CONTROL_PIVOT_LEFT
                            : CAR_CONTROL_PIVOT_RIGHT,
        speedRpm, control->config.car->turnInnerPercent);
    return ANGLE_TURN_RESULT_NONE;
}

void AngleTurnControl_cancel(AngleTurnControl *control)
{
    if (control->active) {
        CarControl_stop(control->config.car);
    }
    control->active = false;
    control->holding = false;
    control->settledCount = 0U;
    control->elapsedSamples = 0U;
}

bool AngleTurnControl_isActive(const AngleTurnControl *control)
{
    return control->active;
}

float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control)
{
    return control->targetYawDegrees -
           AngleTurnControl_getNormalizedYaw(control);
}
