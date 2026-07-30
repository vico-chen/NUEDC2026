#ifndef ANGLE_TURN_CONTROL_H_
#define ANGLE_TURN_CONTROL_H_

#include "car_control.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * MPU6050 相对定角转向状态机
 *
 * 控制流程：定速原地旋转 -> 提前滑行 -> 必要时低速单向补角。
 * 算法不会在过冲后反向追角，避免车体在目标角附近左右抖动。
 */

/* 每次更新返回的状态机结果。 */
typedef enum {
    ANGLE_TURN_RESULT_NONE,
    ANGLE_TURN_RESULT_COMPLETED,
    ANGLE_TURN_RESULT_TIMEOUT,
    ANGLE_TURN_RESULT_FAULT
} AngleTurnControl_Result;

/* 定角算法参数；时间参数均以 10 ms 更新周期为单位。 */
typedef struct {
    CarControl *car;
    /* 手动左转时角度增加填 +1，减小填 -1。 */
    int8_t leftTurnYawSign;
    int16_t defaultCruiseRpm;
    int16_t creepRpm;
    /*
     * 预测滑行角 = clamp(容差 + 增益 × 接近角速度,
     *                    容差, 最大提前角)。
     */
    float brakeRateGain;
    float brakeAheadMaxDegrees;
    /* 判定完成所允许的最终角度误差。 */
    float angleToleranceDegrees;
    float stoppedRateToleranceDps;
    uint16_t settleSamples;
    uint16_t timeoutSamples;
} AngleTurnControl_Config;

/* 定角状态机运行时数据。 */
typedef struct {
    AngleTurnControl_Config config;
    float targetAbsDegrees;
    float turnDirection;
    int16_t cruiseRpm;
    uint16_t settledCount;
    uint16_t coastSamples;
    uint16_t elapsedSamples;
    bool active;
    bool coasting;
    bool creeping;
} AngleTurnControl;

/* 初始化状态机。 */
void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config);
/* 开始一次相对转角控制，并把当前方向重置为 0°。 */
bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t cruiseRpm);
/* 每取得一帧新 MPU 数据后调用一次。 */
AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control);
/* 取消任务并让底盘滑行。 */
void AngleTurnControl_cancel(AngleTurnControl *control);
/* 查询当前是否正在执行定角任务。 */
bool AngleTurnControl_isActive(const AngleTurnControl *control);
/* 返回带方向的剩余角度误差。 */
float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control);

#endif /* ANGLE_TURN_CONTROL_H_ */
