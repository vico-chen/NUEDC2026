#ifndef TASK_EXECUTOR_H_
#define TASK_EXECUTOR_H_

#include "lap_task_control.h"
#include "straight_task_control.h"
#include "task_manager.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 任务执行器依赖的公共硬件和功能模块。
 * 执行器自身不直接控制 GPIO，只负责把选中的任务分发给对应状态机。
 */
typedef struct {
    CarControl *car;                 /* 四轮小车运动控制对象 */
    LineTracking *lineTracking;      /* 八路灰度巡线控制对象 */
    MotorControl *motors[4];         /* A、B、C、D 四个电机对象 */
    void (*log)(const char *text);   /* 串口日志输出回调 */
    bool oledReady;                  /* OLED 是否初始化成功 */
} TaskExecutor_Config;

/* 当前由任务执行器驱动的底层任务模块。 */
typedef enum {
    TASK_EXECUTOR_MODULE_NONE,       /* 当前没有任务运行 */
    TASK_EXECUTOR_MODULE_LAP,        /* 环形巡线一圈模块 */
    TASK_EXECUTOR_MODULE_STRAIGHT    /* 编码器定距直行模块 */
} TaskExecutor_Module;

/* 任务执行器运行状态。 */
typedef struct {
    TaskExecutor_Config config;              /* 公共依赖配置 */
    LapTaskControl lapTask;                  /* 环形任务状态机 */
    StraightTaskControl straightTask;        /* 直行任务状态机 */
    TaskExecutor_Module activeModule;        /* 当前活动模块 */
    TaskManager_Task selectedTask;           /* 按键选中的任务号 */
    uint64_t lastStopwatchDisplayMs;         /* 上次刷新计时器的时刻 */
} TaskExecutor;

/* 初始化、紧急复位、周期调度以及运行状态查询接口。 */
void TaskExecutor_init(TaskExecutor *executor,
    const TaskExecutor_Config *config);
void TaskExecutor_reset(TaskExecutor *executor);
void TaskExecutor_update(TaskExecutor *executor,
    TaskManager_Task selectedTask, bool startPressed);
bool TaskExecutor_isRunning(const TaskExecutor *executor);

#endif /* TASK_EXECUTOR_H_ */
