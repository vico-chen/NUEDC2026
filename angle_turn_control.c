#include "angle_turn_control.h"
#include "mpu6050_angle.h"

/*
 * 定角转向状态机（强调精度并避免反向追角）：
 *   SPIN  -> 锁定方向，以固定巡航 RPM 原地旋转；
 *   COAST -> 根据当前角速度预测惯性角，提前撤销驱动；
 *   CREEP -> 若滑行后仍欠角，则沿原方向低速补角。
 * 发生过冲时不会反向纠正，避免车身在目标角附近来回振荡。
 */

/* 控制算法只需简单绝对值，避免额外依赖数学库。 */
static float AngleTurnControl_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 把传感器角度统一转换为“左转为正”的坐标系。 */
static float AngleTurnControl_getSignedYaw(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZDegrees() *
           (float) control->config.leftTurnYawSign;
}

/* 把传感器角速度统一转换为“左转为正”。 */
static float AngleTurnControl_getSignedRate(
    const AngleTurnControl *control)
{
    return MPU6050_Angle_getZRateDps() *
           (float) control->config.leftTurnYawSign;
}

/* 返回沿命令方向还需要旋转的角度；正数表示仍然欠角。 */
static float AngleTurnControl_getRemainingDegrees(
    const AngleTurnControl *control)
{
    float signedYaw = AngleTurnControl_getSignedYaw(control);
    float signedTarget = control->turnDirection * control->targetAbsDegrees;

    /* remaining 为正表示仍需沿命令方向继续旋转。 */
    return control->turnDirection * (signedTarget - signedYaw);
}

/* 只保留朝目标方向的角速度，背离目标时按 0 处理。 */
static float AngleTurnControl_getApproachRateDps(
    const AngleTurnControl *control)
{
    float approachRate =
        control->turnDirection * AngleTurnControl_getSignedRate(control);

    return (approachRate > 0.0f) ? approachRate : 0.0f;
}

/* 根据实时角速度估计惯性滑行角；转得越快，越早撤销驱动。 */
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

/* 根据 turnDirection 下发左转或右转原地旋转命令。 */
static void AngleTurnControl_commandPivot(
    AngleTurnControl *control, int16_t rpm)
{
    CarControl_setMotion(control->config.car,
        (control->turnDirection > 0.0f) ? CAR_CONTROL_PIVOT_LEFT
                                        : CAR_CONTROL_PIVOT_RIGHT,
        rpm, control->config.car->turnInnerPercent);
}

/* 进入滑行阶段：关闭驱动，并重新开始统计滑行时间。 */
static void AngleTurnControl_enterCoast(AngleTurnControl *control)
{
    CarControl_coast(control->config.car);
    control->coasting = true;
    control->creeping = false;
    control->settledCount = 0U;
    control->coastSamples = 0U;
}

/* 滑行后仍欠角时，进入保持原方向的低速补角阶段。 */
static void AngleTurnControl_enterCreep(AngleTurnControl *control)
{
    control->coasting = false;
    control->creeping = true;
    control->settledCount = 0U;
    control->coastSamples = 0U;
    AngleTurnControl_commandPivot(
        control, control->config.creepRpm);
}

/* 载入配置并把状态机恢复为空闲状态。 */
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

/* 校验参数、清零相对航向并启动一次定角任务。 */
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
    /* 低于 40 RPM 可能无法克服车体静摩擦。 */
    if (cruiseRpm < 40) {
        cruiseRpm = 40;
    }

    /* 开始前撤销旧运动，并把当前位置定义为 0°。 */
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

/* 每 10 ms 根据最新角度/角速度推进一次状态机。 */
AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control)
{
    float remaining;
    float rateAbs;
    float wrongWayLimit;

    if (!control->active) {
        return ANGLE_TURN_RESULT_NONE;
    }

    /* 总超时保护，防止 MPU 异常时无限旋转。 */
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

    /* 误差反而比目标大很多，说明转向符号或安装方向错误。 */
    if (remaining > wrongWayLimit) {
        CarControl_stop(control->config.car);
        control->active = false;
        control->coasting = false;
        control->creeping = false;
        return ANGLE_TURN_RESULT_FAULT;
    }

    /* 已到达或轻微过冲：保持滑行若干帧后判定完成。 */
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

    /* CREEP：仅使用很小的动态提前量，避免再次提前约 2° 停下。 */
    if (control->creeping) {
        float creepStopDistance = control->config.angleToleranceDegrees +
            (0.04f * AngleTurnControl_getApproachRateDps(control));

        AngleTurnControl_commandPivot(control, control->config.creepRpm);
        /* 补角阶段接近目标后再次进入滑行确认。 */
        if (remaining <= creepStopDistance) {
            AngleTurnControl_enterCoast(control);
        }
        return ANGLE_TURN_RESULT_NONE;
    }

    /* COAST：惯性仍在缩小误差时继续等待；基本静止仍欠角则补角。 */
    if (control->coasting) {
        CarControl_coast(control->config.car);
        control->coastSamples++;

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

    /* SPIN：进入预测滑行距离后撤销驱动。 */
    if (remaining <= AngleTurnControl_getCoastDistanceDegrees(control)) {
        AngleTurnControl_enterCoast(control);
        return ANGLE_TURN_RESULT_NONE;
    }

    AngleTurnControl_commandPivot(control, control->cruiseRpm);
    return ANGLE_TURN_RESULT_NONE;
}

/* 取消任务并清除所有阶段计数。 */
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

/* 返回状态机是否处于活动状态。 */
bool AngleTurnControl_isActive(const AngleTurnControl *control)
{
    return control->active;
}

/* 返回沿命令方向的剩余角度。 */
float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control)
{
    return AngleTurnControl_getRemainingDegrees(control);
}
