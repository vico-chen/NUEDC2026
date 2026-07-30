#ifndef LAP_TASK_CONTROL_H_
#define LAP_TASK_CONTROL_H_

#include "line_tracking.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

/* 所有 samples 参数都以统一的 10 ms 任务节拍为单位。 */
typedef struct {
    int16_t cruiseRpm;
    int16_t straightRpm;
    uint16_t accelerationSamples;
    uint16_t straightAccelerationSamples;
    uint8_t intersectionActiveThreshold;
    uint8_t intersectionConfirmSamples;
    uint16_t finishAdvanceMm;
    uint16_t wheelDiameterMm;
    int16_t decelerationEndRpm;
    uint16_t decelerationSamples;
    uint16_t brakeMinimumSamples;
    uint16_t brakeTimeoutSamples;
} LapTask_Profile;

typedef struct {
    CarControl *car;
    LineTracking *lineTracking;
    MotorControl *motors[4];
    void (*log)(const char *text);
} LapTask_Config;

typedef enum {
    LAP_TASK_IDLE,
    LAP_TASK_FOLLOW,
    LAP_TASK_FINISH_ADVANCE,
    LAP_TASK_DECELERATE,
    LAP_TASK_BRAKE,
    LAP_TASK_DONE,
    LAP_TASK_FAULT
} LapTask_State;

typedef struct {
    LapTask_Config config;
    const LapTask_Profile *profile;
    LapTask_State state;
    uint8_t taskNumber;
    uint8_t intersectionConfirmCount;
    uint16_t accelerationSamples;
    uint16_t straightAccelerationSamples;
    uint16_t decelerationSamples;
    uint16_t brakeSamples;
    int16_t finishStartRpm;
    CarControl_Motion finishMotion;
    uint8_t finishInnerPercent;
    uint32_t finishEncoderCount;
    uint32_t finishTargetEncoderCount;
} LapTaskControl;

void LapTaskControl_init(LapTaskControl *control,
    const LapTask_Config *config);
bool LapTaskControl_start(LapTaskControl *control, uint8_t taskNumber,
    const LapTask_Profile *profile);
void LapTaskControl_reset(LapTaskControl *control);
void LapTaskControl_update(LapTaskControl *control);
bool LapTaskControl_isRunning(const LapTaskControl *control);
bool LapTaskControl_isFinished(const LapTaskControl *control);

#endif /* LAP_TASK_CONTROL_H_ */
