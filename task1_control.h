#ifndef TASK1_CONTROL_H_
#define TASK1_CONTROL_H_

#include "angle_turn_control.h"
#include "motor_control.h"
#include "task_manager.h"
#include "stopwatch.h"
#include "oled.h"

#include <stdbool.h>
#include <stdint.h>

/* Task1 对底层车辆、转向、巡线、灰度和日志模块的统一接口。 */
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

/* Task1 完整运行流程的状态。 */
typedef enum {
    TASK1_CONTROL_WAIT_LOAD,           /* 等待装载完成按钮 */
    TASK1_CONTROL_FOLLOW_INTERSECTION, /* 巡线寻找目标十字路口 */
    TASK1_CONTROL_ADVANCE,             /* 进入路口中心 */
    TASK1_CONTROL_BRAKE_TURN,          /* 转向前制动 */
    TASK1_CONTROL_TURNING,             /* 执行 85 度转向 */
    TASK1_CONTROL_FOLLOW_BLANK,        /* 转向后巡线寻找终点空白 */
    TASK1_CONTROL_END_BRAKE,           /* 到达终点后制动 */
    TASK1_CONTROL_DONE,                /* 往返任务完成 */
    TASK1_CONTROL_FAULT                /* 传感器、转向或超时故障 */
} Task1Control_State;

/* Task1 状态机的计数器和路线标志。 */
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
    uint64_t lastShowStopwatchSystick;
} Task1Control;

/* 初始化、复位、周期推进以及运行状态查询。 */
void Task1Control_init(Task1Control *control,
    const Task1Control_Config *config);
void Task1Control_reset(Task1Control *control);
void Task1Control_update(Task1Control *control, TaskManager_Task activeTask,
    TaskManager_Task1Endpoint endpoint, bool startTaskPressed, bool mpu6050Ready,
    AngleTurnControl_Result angleTurnResult);
bool Task1Control_isActive(const Task1Control *control);

#endif /* TASK1_CONTROL_H_ */
