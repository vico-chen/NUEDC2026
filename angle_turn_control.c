#include "angle_turn_control.h"
#include "mpu6050_angle.h"

/*
 * 连续定角转向：
 *   按固定巡航转速持续旋转，达到目标角度后立即关闭驱动并返回完成。
 * 不再提前滑行、等待角速度下降或低速补角。
 */

/* 应用安装方向标定，使“左转角度为正”的约定保持一致。 */
static float AngleTurnControl_getSignedYaw(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZDegrees() *
           (float) control->config.leftTurnYawSign;
}

/* 计算沿当前命令方向尚未完成的角度。 */
static float AngleTurnControl_getRemainingDegrees(
    const AngleTurnControl *control)
{
    float signedYaw = AngleTurnControl_getSignedYaw(control);
    float signedTarget = control->turnDirection * control->targetAbsDegrees;

    /* 返回正数表示仍需继续旋转，0 或负数表示已经到达或过冲。 */
    return control->turnDirection * (signedTarget - signedYaw);
}

/* 将左/右方向转换成车辆原地旋转命令。 */
static void AngleTurnControl_commandPivot(
    AngleTurnControl *control, int16_t rpm)
{
    CarControl_setMotion(control->config.car,
        (control->turnDirection > 0.0f) ? CAR_CONTROL_PIVOT_LEFT
                                        : CAR_CONTROL_PIVOT_RIGHT,
        rpm, control->config.car->turnInnerPercent);
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
    wrongWayLimit = control->targetAbsDegrees + 25.0f;

    if (remaining > wrongWayLimit) {
        CarControl_stop(control->config.car);
        control->active = false;
        control->coasting = false;
        control->creeping = false;
        return ANGLE_TURN_RESULT_FAULT;
    }

    /* 到达容差或已经过冲时立即关闭驱动，不再滑行等待或低速补角。 */
    if (remaining <= control->config.angleToleranceDegrees) {
        CarControl_coast(control->config.car);
        control->active = false;
        control->coasting = false;
        control->creeping = false;
        control->settledCount = 0U;
        control->coastSamples = 0U;
        return ANGLE_TURN_RESULT_COMPLETED;
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
