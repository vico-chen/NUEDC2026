#ifndef TASK_EXECUTOR_H_
#define TASK_EXECUTOR_H_

#include "lap_task_control.h"
#include "straight_task_control.h"
#include "task_manager.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    CarControl *car;
    LineTracking *lineTracking;
    MotorControl *motors[4];
    void (*log)(const char *text);
    void (*logInt32)(int32_t value);
    /* 指向主程序的 OLED 在线标志；通信失败后统一置 false。 */
    bool *oledReady;
} TaskExecutor_Config;

typedef enum {
    TASK_MODULE_NONE,
    TASK_MODULE_LAP,
    TASK_MODULE_STRAIGHT,
    /* Task3 暂无车辆控制，仅用按键/QS 开始和停止秒表。 */
    TASK_MODULE_TIMER
} TaskExecutor_Module;

typedef struct {
    TaskExecutor_Config config;
    LapTaskControl lapTask;
    StraightTaskControl straightTask;
    TaskExecutor_Module activeModule;
    TaskId activeTask;
    bool timerTaskRunning;
    bool lastTaskFault;
    uint32_t lastOledRefreshMs;
} TaskExecutor;

void TaskExecutor_init(TaskExecutor *executor,
    const TaskExecutor_Config *config);
void TaskExecutor_reset(TaskExecutor *executor);
bool TaskExecutor_startSelected(TaskExecutor *executor);
void TaskExecutor_update10ms(TaskExecutor *executor);
bool TaskExecutor_isRunning(const TaskExecutor *executor);
TaskId TaskExecutor_getActiveTask(const TaskExecutor *executor);
bool TaskExecutor_lastTaskFault(const TaskExecutor *executor);

#endif /* TASK_EXECUTOR_H_ */
