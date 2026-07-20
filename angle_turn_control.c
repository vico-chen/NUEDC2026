#include "angle_turn_control.h"
#include "mpu6050_angle.h"

/*
 * Accuracy-focused pivot profile without reverse hunting:
 *   SPIN  -> fixed cruise RPM, locked direction
 *   COAST -> start from rate-predicted remaining angle; no reverse brake
 *   CREEP -> if coast stops short, slow one-way finish to tolerance
 * Never reverse to correct overshoot.
 */

static float AngleTurnControl_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float AngleTurnControl_getSignedYaw(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZDegrees() *
           (float) control->config.leftTurnYawSign;
}

static float AngleTurnControl_getSignedRate(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZRateDps() *
           (float) control->config.leftTurnYawSign;
}

static float AngleTurnControl_getRemainingDegrees(
    const AngleTurnControl *control)
{
    float signedYaw = AngleTurnControl_getSignedYaw(control);
    float signedTarget = control->turnDirection * control->targetAbsDegrees;

    /* Positive remaining => still need more rotation in the command direction. */
    return control->turnDirection * (signedTarget - signedYaw);
}

static float AngleTurnControl_getApproachRateDps(
    const AngleTurnControl *control)
{
    float approachRate =
        control->turnDirection * AngleTurnControl_getSignedRate(control);

    return (approachRate > 0.0f) ? approachRate : 0.0f;
}

static float AngleTurnControl_getCoastDistanceDegrees(
    const AngleTurnControl *control)
{
    float distance =
        control->config.angleToleranceDegrees +
        control->config.brakeRateGain *
            AngleTurnControl_getApproachRateDps(control);

    if (distance < control->config.angleToleranceDegrees) {
        distance = control->config.angleToleranceDegrees;
    }
    if (distance > control->config.brakeAheadMaxDegrees) {
        distance = control->config.brakeAheadMaxDegrees;
    }
    return distance;
}

static void AngleTurnControl_commandPivot(
    AngleTurnControl *control, int16_t rpm)
{
    CarControl_setMotion(control->config.car,
        (control->turnDirection > 0.0f) ? CAR_CONTROL_PIVOT_LEFT
                                        : CAR_CONTROL_PIVOT_RIGHT,
        rpm, control->config.car->turnInnerPercent);
}

static void AngleTurnControl_enterCoast(AngleTurnControl *control)
{
    CarControl_coast(control->config.car);
    control->coasting = true;
    control->creeping = false;
    control->settledCount = 0U;
    control->coastSamples = 0U;
}

static void AngleTurnControl_enterCreep(AngleTurnControl *control)
{
    control->coasting = false;
    control->creeping = true;
    control->settledCount = 0U;
    control->coastSamples = 0U;
    AngleTurnControl_commandPivot(
        control, control->config.creepRpm);
}

void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config)
{
    control->config = *config;
    control->targetAbsDegrees = 0.0f;
    control->turnDirection = 1.0f;
    control->cruiseRpm = config->defaultCruiseRpm;
    control->settledCount = 0U;
    control->coastSamples = 0U;
    control->elapsedSamples = 0U;
    control->active = false;
    control->coasting = false;
    control->creeping = false;
}

bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t cruiseRpm)
{
    if (relativeAngleDegrees <= 0.0f) {
        return false;
    }
    if (cruiseRpm <= 0) {
        cruiseRpm = control->config.defaultCruiseRpm;
    }
    if (cruiseRpm > control->config.car->config.maximumSpeedRpm) {
        cruiseRpm = control->config.car->config.maximumSpeedRpm;
    }
    if (cruiseRpm < 40) {
        cruiseRpm = 40;
    }

    CarControl_coast(control->config.car);
    MPU6050_Angle_reset();

    control->targetAbsDegrees = relativeAngleDegrees;
    control->turnDirection = turnLeft ? 1.0f : -1.0f;
    control->cruiseRpm = cruiseRpm;
    control->settledCount = 0U;
    control->coastSamples = 0U;
    control->elapsedSamples = 0U;
    control->coasting = false;
    control->creeping = false;
    control->active = true;

    AngleTurnControl_commandPivot(control, cruiseRpm);
    return true;
}

AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control)
{
    float remaining;
    float rateAbs;
    float wrongWayLimit;

    if (!control->active) {
        return ANGLE_TURN_RESULT_NONE;
    }

    control->elapsedSamples++;
    if (control->elapsedSamples >= control->config.timeoutSamples) {
        CarControl_stop(control->config.car);
        control->active = false;
        control->coasting = false;
        control->creeping = false;
        return ANGLE_TURN_RESULT_TIMEOUT;
    }

    remaining = AngleTurnControl_getRemainingDegrees(control);
    rateAbs = AngleTurnControl_abs(AngleTurnControl_getSignedRate(control));
    wrongWayLimit = control->targetAbsDegrees + 25.0f;

    if (remaining > wrongWayLimit) {
        CarControl_stop(control->config.car);
        control->active = false;
        control->coasting = false;
        control->creeping = false;
        return ANGLE_TURN_RESULT_FAULT;
    }

    /* Reached or slightly overshot: coast briefly, then DONE. */
    if (remaining <= control->config.angleToleranceDegrees) {
        if (!control->coasting) {
            AngleTurnControl_enterCoast(control);
            control->settledCount = 1U;
            return ANGLE_TURN_RESULT_NONE;
        }
        CarControl_coast(control->config.car);
        control->settledCount++;
        if (control->settledCount >= control->config.settleSamples) {
            control->active = false;
            control->coasting = false;
            control->creeping = false;
            return ANGLE_TURN_RESULT_COMPLETED;
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    if (control->creeping) {
        float creepStopDistance = control->config.angleToleranceDegrees +
            (0.04f * AngleTurnControl_getApproachRateDps(control));

        AngleTurnControl_commandPivot(control, control->config.creepRpm);
        /*
         * During creep, do not reuse the large cruise coast distance; that
         * left a repeatable ~2 deg shortfall. Stop only when nearly there.
         */
        if (remaining <= creepStopDistance) {
            AngleTurnControl_enterCoast(control);
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    if (control->coasting) {
        CarControl_coast(control->config.car);
        control->coastSamples++;

        /*
         * Keep coasting while inertia is still closing the gap.
         * If almost stopped and still short, finish with a slow creep.
         */
        if ((rateAbs <= control->config.stoppedRateToleranceDps) &&
            (control->coastSamples >= 10U)) {
            AngleTurnControl_enterCreep(control);
            return ANGLE_TURN_RESULT_NONE;
        }
        if (control->coastSamples >= 150U) {
            AngleTurnControl_enterCreep(control);
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    if (remaining <= AngleTurnControl_getCoastDistanceDegrees(control)) {
        AngleTurnControl_enterCoast(control);
        return ANGLE_TURN_RESULT_NONE;
    }

    AngleTurnControl_commandPivot(control, control->cruiseRpm);
    return ANGLE_TURN_RESULT_NONE;
}

void AngleTurnControl_cancel(AngleTurnControl *control)
{
    if (control->active) {
        CarControl_stop(control->config.car);
    }
    control->active = false;
    control->coasting = false;
    control->creeping = false;
    control->settledCount = 0U;
    control->coastSamples = 0U;
    control->elapsedSamples = 0U;
}

bool AngleTurnControl_isActive(const AngleTurnControl *control)
{
    return control->active;
}

float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control)
{
    return AngleTurnControl_getRemainingDegrees(control);
}
