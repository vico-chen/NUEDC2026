#ifndef ANGLE_TURN_CONTROL_H_
#define ANGLE_TURN_CONTROL_H_

#include "car_control.h"

#include <stdbool.h>
#include <stdint.h>

/* 一次定角转向更新的执行结果。 */
typedef enum {
    ANGLE_TURN_RESULT_NONE,
    ANGLE_TURN_RESULT_COMPLETED,
    ANGLE_TURN_RESULT_TIMEOUT,
    ANGLE_TURN_RESULT_FAULT
} AngleTurnControl_Result;

/* 定角转向所需的车辆对象、方向标定和停止判据。 */
typedef struct {
    CarControl *car;
    int8_t leftTurnYawSign;
    int16_t defaultCruiseRpm;
    /* 以下滑行/补角参数仅为兼容现有配置保留，连续模式不再使用。 */
    int16_t creepRpm;
    float brakeRateGain;
    float brakeAheadMaxDegrees;
    /* 达到该角度误差后立即关闭驱动并判定转向完成。 */
    float angleToleranceDegrees;
    float stoppedRateToleranceDps;
    uint16_t settleSamples;
    uint16_t timeoutSamples;
} AngleTurnControl_Config;

/* 当前转向的目标、计数器与运行标志。 */
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

/* 初始化、启动、周期更新、取消以及查询定角转向。 */
void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config);
bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t cruiseRpm);
AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control);
void AngleTurnControl_cancel(AngleTurnControl *control);
bool AngleTurnControl_isActive(const AngleTurnControl *control);
float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control);

#endif /* ANGLE_TURN_CONTROL_H_ */
