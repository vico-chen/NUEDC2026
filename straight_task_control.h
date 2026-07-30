#ifndef STRAIGHT_TASK_CONTROL_H_
#define STRAIGHT_TASK_CONTROL_H_

#include "car_control.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t targetDistanceMm;
    uint16_t wheelDiameterMm;
    int16_t cruiseRpm;
    uint16_t accelerationSamples;
    uint16_t decelerationDistanceMm;
    int16_t decelerationEndRpm;
    uint16_t brakeMinimumSamples;
    uint16_t brakeTimeoutSamples;
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
    uint16_t brakeSamples;
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
