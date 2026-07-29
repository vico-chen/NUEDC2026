#ifndef TASK1_CONTROL_H_
#define TASK1_CONTROL_H_

#include "angle_turn_control.h"
#include "motor_control.h"
#include "task_manager.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    CarControl *car;
    AngleTurnControl *angleTurn;
    MotorControl *motors[4];
    void (*setLineTrackingEnabled)(bool enabled);
    void (*setLineTrackingSpeed)(int16_t speedRpm);
    void (*resetLineTracking)(void);
    void (*updateLineTracking)(void);
    uint8_t (*readActiveChannelCount)(void);
    uint8_t (*readActiveChannelMask)(void);
    void (*log)(const char *text);
} Task1Control_Config;

typedef enum {
    TASK1_CONTROL_WAIT_LOAD,
    TASK1_CONTROL_FOLLOW_INTERSECTION,
    TASK1_CONTROL_ADVANCE,
    TASK1_CONTROL_BRAKE_TURN,
    TASK1_CONTROL_TURNING,
    TASK1_CONTROL_FOLLOW_BLANK,
    TASK1_CONTROL_END_BRAKE,
    TASK1_CONTROL_END_REVERSE,
    TASK1_CONTROL_FINAL_BRAKE,
    TASK1_CONTROL_WAIT_UNLOAD,
    TASK1_CONTROL_RETURN_TURNING,
    TASK1_CONTROL_DONE,
    TASK1_CONTROL_FAULT
} Task1Control_State;

typedef struct {
    Task1Control_Config config;
    Task1Control_State state;
    uint8_t intersectionPartialCount;
    uint8_t advanceSamples;
    uint8_t accelerationSamples;
    uint8_t blankCount;
    uint16_t brakeSamples;
    uint8_t reverseSamples;
    uint8_t finalBrakeSettledCount;
    bool lineSeenAfterTurn;
    bool endpoint2;
    bool returnPhase;
    bool nextTurnRight;
} Task1Control;

void Task1Control_init(Task1Control *control,
    const Task1Control_Config *config);
void Task1Control_reset(Task1Control *control);
void Task1Control_update(Task1Control *control, TaskManager_Task activeTask,
    TaskManager_Task1Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool mpu6050Ready,
    AngleTurnControl_Result angleTurnResult);
bool Task1Control_isActive(const Task1Control *control);

#endif /* TASK1_CONTROL_H_ */
