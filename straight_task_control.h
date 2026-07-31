#ifndef STRAIGHT_TASK_CONTROL_H_
#define STRAIGHT_TASK_CONTROL_H_

#include "car_control.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t targetDistanceMm;       /* 编码器标定后的停车目标距离 */
    uint16_t wheelDiameterMm;        /* 轮胎有效直径 */
    int16_t cruiseRpm;               /* 中段巡航目标速度 */
    uint16_t accelerationSamples;    /* 0 到巡航速度的 10 ms 周期数 */
    uint16_t decelerationDistanceMm; /* 距目标多远开始减速 */
    uint16_t decelerationSamples;    /* 时间匀减速的 10 ms 周期数 */
    int16_t decelerationEndRpm;      /* 斜坡结束后的低速爬行速度 */
    uint16_t brakeMinimumSamples;    /* 停车后最少确认时间 */
    uint16_t brakeTimeoutSamples;    /* 停车确认超时时间 */
} StraightTask_Profile;

typedef struct {
    CarControl *car;
    MotorControl *motors[4];
    void (*log)(const char *text);
} StraightTask_Config;

typedef enum {
    STRAIGHT_TASK_IDLE,
    STRAIGHT_TASK_RUNNING,
    STRAIGHT_TASK_BRAKE,
    STRAIGHT_TASK_DONE,
    STRAIGHT_TASK_FAULT
} StraightTask_State;

typedef struct {
    StraightTask_Config config;
    const StraightTask_Profile *profile;
    StraightTask_State state;
    uint16_t accelerationSamples;
    uint16_t decelerationSamples;
    uint16_t brakeSamples;
    int16_t commandedRpm;          /* 上一周期实际下发速度 */
    int16_t decelerationStartRpm;  /* 进入减速区时保存的速度 */
    bool decelerationStarted;      /* 是否已经进入时间减速区 */
    uint32_t encoderCount;
    uint32_t targetEncoderCount;
    uint32_t decelerationEncoderCount;
} StraightTaskControl;

void StraightTaskControl_init(StraightTaskControl *control,
    const StraightTask_Config *config);
bool StraightTaskControl_start(StraightTaskControl *control,
    const StraightTask_Profile *profile);
void StraightTaskControl_reset(StraightTaskControl *control);
void StraightTaskControl_update(StraightTaskControl *control);
bool StraightTaskControl_isRunning(const StraightTaskControl *control);
bool StraightTaskControl_isFinished(const StraightTaskControl *control);

#endif /* STRAIGHT_TASK_CONTROL_H_ */
