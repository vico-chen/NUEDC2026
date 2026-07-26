#ifndef TASK3_CONTROL_H_
#define TASK3_CONTROL_H_

#include "task1_control.h"

typedef enum {
    TASK3_TURN_NONE,
    TASK3_TURN_LEFT,
    TASK3_TURN_RIGHT
} Task3Control_Turn;

typedef enum {
    TASK3_WAIT_INFO,
    TASK3_WAIT_LOAD,
    TASK3_FOLLOW,
    TASK3_ADVANCE,
    TASK3_BRAKE_TURN,
    TASK3_TURNING,
    TASK3_BRAKE_END,
    TASK3_REVERSE,
    TASK3_FINAL_BRAKE,
    TASK3_WAIT_UNLOAD,
    TASK3_RETURN_FOLLOW,
    TASK3_DONE,
    TASK3_FAULT
} Task3Control_State;

typedef struct {
    Task1Control_Config io;
    Task3Control_State state;
    Task3Control_Turn pendingTurn;
    uint8_t completedTurns;
    uint8_t completedReturnTurns;
    uint8_t rampSamples;
    uint8_t intersectionConfirmSamples;
    uint8_t advanceSamples;
    uint8_t blankSamples;
    uint8_t reverseSamples;
    uint8_t settledSamples;
    uint16_t brakeSamples;
    bool lineSeenAfterFinalTurn;
    bool intersectionArmed;
    bool returning;
    bool uTurn;
} Task3Control;

void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io);
void Task3Control_reset(Task3Control *control);
void Task3Control_update(Task3Control *control, TaskManager_Task task,
    bool statusPressed, bool statusReleased, bool numberReceived,
    Task3Control_Turn visualTurn, bool mpuReady,
    AngleTurnControl_Result turnResult);

#endif /* TASK3_CONTROL_H_ */
