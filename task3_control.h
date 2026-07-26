#ifndef TASK3_CONTROL_H_
#define TASK3_CONTROL_H_

#include "task1_control.h"

typedef enum {
    TASK3_WAIT_LOAD,
    TASK3_FOLLOW_THIRD_INTERSECTION,
    TASK3_FOLLOW_NEXT_INTERSECTION,
    TASK3_ADVANCE,
    TASK3_BRAKE_TURN,
    TASK3_TURNING,
    TASK3_FOLLOW_BLANK,
    TASK3_END_BRAKE,
    TASK3_REVERSE,
    TASK3_FINAL_BRAKE,
    TASK3_WAIT_UNLOAD,
    TASK3_RETURN_TURNING,
    TASK3_RETURN_INTERSECTION,
    TASK3_DONE,
    TASK3_FAULT
} Task3Control_State;

typedef struct {
    Task1Control_Config io;
    Task3Control_State state;
    TaskManager_Task3Endpoint endpoint;
    uint8_t outboundCrossCount;
    uint8_t completedOutboundTurns;
    uint8_t completedReturnTurns;
    uint8_t intersectionConfirmCount;
    uint8_t advanceSamples;
    uint8_t rampSamples;
    uint8_t blankSamples;
    uint8_t reverseSamples;
    uint8_t settledSamples;
    uint16_t brakeSamples;
    bool returnPhase;
    bool nextTurnRight;
    bool intersectionArmed;
    bool lineSeenAfterTurn;
} Task3Control;

void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io);
void Task3Control_reset(Task3Control *control);
void Task3Control_update(Task3Control *control,
    TaskManager_Task activeTask, TaskManager_Task3Endpoint endpoint,
    bool statusPressed, bool statusReleased, bool mpuReady,
    AngleTurnControl_Result turnResult);
bool Task3Control_isActive(const Task3Control *control);

#endif /* TASK3_CONTROL_H_ */
