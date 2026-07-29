#ifndef TASK3_CONTROL_H_
#define TASK3_CONTROL_H_

#include "task1_control.h"
#include "task2_control.h"

/* Task3 需要连续完成两次路口转向，其余停车阶段与 Task2 相同。 */
typedef enum {
    TASK3_WAIT_INFO,     /* AUTO 模式等待 OpenMV 数字 */
    TASK3_WAIT_LOAD,     /* 等待装载完成 */
    TASK3_FOLLOW,        /* 去程巡线并执行两次转向 */
    TASK3_ADVANCE,       /* 进入当前路口中心 */
    TASK3_BRAKE_TURN,    /* 转向前制动 */
    TASK3_TURNING,       /* 执行路口转向或掉头 */
    TASK3_BRAKE_END,     /* 终点制动 */
    TASK3_REVERSE,       /* 终点倒车调整 */
    TASK3_FINAL_BRAKE,   /* 最终停车并等待稳定 */
    TASK3_WAIT_UNLOAD,   /* 等待卸载完成 */
    TASK3_RETURN_FOLLOW, /* 返程巡线并反向还原路线 */
    TASK3_DONE,          /* 往返完成 */
    TASK3_FAULT          /* 转向失败或停车超时 */
} Task3Control_State;

/* Task3 状态机的两段路线、路口计数器和运行标志。 */
typedef struct {
    Task1Control_Config io;
    Task3Control_State state;
    Task2Control_Turn pendingTurn;
    uint8_t completedTurns;
    uint8_t completedReturnTurns;
    uint8_t outboundIntersectionsSeen;
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
    bool visionNumberReady;
    TaskManager_Task3Endpoint selectedEndpoint;
} Task3Control;

/* 初始化、复位并按 10 ms 周期推进 Task3。 */
void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io);
void Task3Control_reset(Task3Control *control);
void Task3Control_update(Task3Control *control, TaskManager_Task task,
    TaskManager_Task3Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool numberReceived,
    Task2Control_Turn visualTurn, bool mpuReady,
    AngleTurnControl_Result turnResult);

#endif /* TASK3_CONTROL_H_ */
