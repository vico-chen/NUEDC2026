#ifndef TASK2_CONTROL_H_
#define TASK2_CONTROL_H_

#include "task1_control.h"

/* OpenMV 或端点预设给出的下一次路口转向命令。 */
typedef enum {
    TASK2_TURN_NONE,
    TASK2_TURN_LEFT,
    TASK2_TURN_RIGHT
} Task2Control_Turn;

/* Task2 状态机各阶段。 */
typedef enum {
    TASK2_WAIT_INFO,    /* AUTO 模式等待 OpenMV 数字 */
    TASK2_WAIT_LOAD,    /* 路线已知，等待装载完成 */
    TASK2_FOLLOW,       /* 去程巡线并统计十字路口 */
    TASK2_ADVANCE,      /* 进入路口中心 */
    TASK2_BRAKE_TURN,   /* 转向前制动 */
    TASK2_TURNING,      /* 执行 85 度转向或 180 度掉头 */
    TASK2_BRAKE_END,    /* 终点制动 */
    TASK2_REVERSE,      /* 终点倒车调整 */
    TASK2_FINAL_BRAKE,  /* 最终停车并等待稳定 */
    TASK2_WAIT_UNLOAD,  /* 等待卸载完成 */
    TASK2_RETURNING,    /* 返程巡线 */
    TASK2_DONE,         /* 往返完成 */
    TASK2_FAULT         /* 转向失败或停车超时 */
} Task2Control_State;

/* Task2 的状态、路线选择、路口计数和时间计数器。 */
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

/* 初始化、复位、周期推进以及运行状态查询。 */
void Task2Control_init(Task2Control *control, const Task1Control_Config *io);
void Task2Control_reset(Task2Control *control);
void Task2Control_update(Task2Control *control, TaskManager_Task task,
    TaskManager_Task2Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool numberReceived,
    Task2Control_Turn visualTurn, bool mpuReady, AngleTurnControl_Result turnResult);
bool Task2Control_isActive(const Task2Control *control);

#endif
