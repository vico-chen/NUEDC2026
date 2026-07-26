#ifndef TASK2_CONTROL_H_
#define TASK2_CONTROL_H_

#include "task1_control.h"

typedef enum {
    TASK2_WAIT_LOAD,
    TASK2_FOLLOW_SECOND_INTERSECTION,
    TASK2_ADVANCE,
    TASK2_BRAKE_TURN,
    TASK2_TURNING,
    TASK2_FOLLOW_BLANK,
    TASK2_END_BRAKE,
    TASK2_REVERSE,
    TASK2_FINAL_BRAKE,
    TASK2_WAIT_UNLOAD,
    TASK2_RETURN_TURNING,
    TASK2_RETURN_INTERSECTION,
    TASK2_DONE,
    TASK2_FAULT
} Task2Control_State;

typedef struct {
    Task1Control_Config io;
    Task2Control_State state;
    uint8_t outboundCrossCount;
    uint8_t intersectionConfirmCount;
    uint8_t advanceSamples;
    uint8_t rampSamples;
    uint8_t blankSamples;
    uint8_t reverseSamples;
    uint8_t settledSamples;
    uint16_t brakeSamples;
    bool endpoint2;
    bool returnPhase;
    bool nextTurnRight;
    bool intersectionArmed;
    bool lineSeenAfterTurn;
} Task2Control;

void Task2Control_init(Task2Control *control,
    const Task1Control_Config *io);
void Task2Control_reset(Task2Control *control);
void Task2Control_update(Task2Control *control, TaskManager_Task activeTask,
    TaskManager_Task1Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool mpuReady,
    AngleTurnControl_Result turnResult);
bool Task2Control_isActive(const Task2Control *control);

#endif /* TASK2_CONTROL_H_ */
