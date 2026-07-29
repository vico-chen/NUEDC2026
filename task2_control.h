#ifndef TASK2_CONTROL_H_
#define TASK2_CONTROL_H_

#include "task1_control.h"

typedef enum { TASK2_TURN_NONE, TASK2_TURN_LEFT, TASK2_TURN_RIGHT } Task2Control_Turn;
typedef enum { TASK2_WAIT_INFO, TASK2_WAIT_LOAD, TASK2_FOLLOW, TASK2_ADVANCE,
    TASK2_BRAKE_TURN, TASK2_TURNING, TASK2_BRAKE_END, TASK2_REVERSE,
    TASK2_FINAL_BRAKE, TASK2_WAIT_UNLOAD, TASK2_RETURNING, TASK2_DONE,
    TASK2_FAULT } Task2Control_State;
typedef struct {
    Task1Control_Config io;
    Task2Control_State state;
    Task2Control_Turn pendingTurn;
    Task2Control_Turn outboundTurn;
    TaskManager_Task2Endpoint selectedEndpoint;
    uint8_t ramp, intersectionCount, outboundIntersectionsSeen;
    uint8_t advance, blank, reverse, settle;
    uint16_t brake;
    bool lineSeen, intersectionLatched, returning, numberReceived;
    bool turnCompletedThisLeg, isUTurn;
} Task2Control;

void Task2Control_init(Task2Control *control, const Task1Control_Config *io);
void Task2Control_reset(Task2Control *control);
void Task2Control_update(Task2Control *control, TaskManager_Task task,
    TaskManager_Task2Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool numberReceived,
    Task2Control_Turn visualTurn, bool mpuReady, AngleTurnControl_Result turnResult);
bool Task2Control_isActive(const Task2Control *control);

#endif
