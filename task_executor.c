#include "task_executor.h"

#include "oled.h"
#include "stopwatch.h"
#include "system_time.h"
#include "task_profiles.h"

#define TASK_OLED_REFRESH_MS (1000U)

static void TaskExecutor_log(
    const TaskExecutor *executor, const char *text)
{
    if (executor->config.log != 0) {
        executor->config.log(text);
    }
}

static const LapTask_Profile *TaskExecutor_lapProfile(TaskId task)
{
    switch (task) {
        case TASK_ID_2:
            return &gLapTask2Profile;
        case TASK_ID_5:
            return &gLapTask5Profile;
        case TASK_ID_6:
            return &gLapTask6Profile;
        default:
            return 0;
    }
}

static void TaskExecutor_stopModules(TaskExecutor *executor)
{
    LapTaskControl_reset(&executor->lapTask);
    StraightTaskControl_reset(&executor->straightTask);
    executor->timerTaskRunning = false;
    executor->activeModule = TASK_MODULE_NONE;
}

static bool TaskExecutor_oledAvailable(const TaskExecutor *executor)
{
    return (executor->config.oledReady != 0) &&
           (*executor->config.oledReady);
}

static void TaskExecutor_showStopwatch(TaskExecutor *executor)
{
    if (TaskExecutor_oledAvailable(executor)) {
        *executor->config.oledReady = OLED_ShowStopwatch(
            (uint8_t) executor->activeTask,
            Stopwatch_getElapsedMs());
    }
}

static void TaskExecutor_showResult(TaskExecutor *executor,
    OLED_TaskResult result)
{
    if (TaskExecutor_oledAvailable(executor)) {
        *executor->config.oledReady = OLED_ShowTaskResult(
            (uint8_t) executor->activeTask, result,
            Stopwatch_getElapsedMs());
    }
}

static void TaskExecutor_logElapsed(TaskExecutor *executor)
{
    TaskExecutor_log(executor, "TASK_ELAPSED_MS=");
    if (executor->config.logInt32 != 0) {
        executor->config.logInt32(
            (int32_t) Stopwatch_getElapsedMs());
    }
    TaskExecutor_log(executor, "\r\n");
}

void TaskExecutor_init(TaskExecutor *executor,
    const TaskExecutor_Config *config)
{
    LapTask_Config lapConfig;
    StraightTask_Config straightConfig;
    uint8_t i;

    *executor = (TaskExecutor) {0};
    executor->config = *config;

    lapConfig.car = config->car;
    lapConfig.lineTracking = config->lineTracking;
    lapConfig.log = config->log;
    straightConfig.car = config->car;
    straightConfig.log = config->log;
    for (i = 0U; i < 4U; i++) {
        lapConfig.motors[i] = config->motors[i];
        straightConfig.motors[i] = config->motors[i];
    }
    LapTaskControl_init(&executor->lapTask, &lapConfig);
    StraightTaskControl_init(&executor->straightTask, &straightConfig);
    executor->activeTask = TASK_ID_2;
    executor->lastOledRefreshMs = SystemTime_getMs();
    Stopwatch_reset();
}

void TaskExecutor_reset(TaskExecutor *executor)
{
    bool wasRunning = TaskExecutor_isRunning(executor);

    TaskExecutor_stopModules(executor);
    Stopwatch_stop();
    if (wasRunning) {
        TaskExecutor_showResult(
            executor, OLED_TASK_RESULT_CANCELLED);
    }
}

bool TaskExecutor_startSelected(TaskExecutor *executor)
{
    TaskId selected = TaskManager_getSelected();
    const LapTask_Profile *lapProfile =
        TaskExecutor_lapProfile(selected);
    bool started = false;

    /*
     * Task3 是纯计时占位任务：第一次 PA29/QS 启动，第二次结束。
     * 在这里完成切换，使实体按键和 UART 命令保持完全相同的行为。
     */
    if (TaskExecutor_isRunning(executor) &&
        (executor->activeModule == TASK_MODULE_TIMER) &&
        (executor->activeTask == TASK_ID_3)) {
        executor->timerTaskRunning = false;
        Stopwatch_stop();
        TaskExecutor_log(executor, "TASK3_TIMER_STOPPED\r\n");
        TaskExecutor_logElapsed(executor);
        TaskExecutor_showResult(executor, OLED_TASK_RESULT_DONE);
        return true;
    }
    if (TaskExecutor_isRunning(executor)) {
        TaskExecutor_log(executor, "TASK_ALREADY_RUNNING\r\n");
        return false;
    }
    TaskExecutor_stopModules(executor);
    executor->activeTask = selected;
    executor->lastTaskFault = false;

    if (lapProfile != 0) {
        started = LapTaskControl_start(&executor->lapTask,
            (uint8_t) selected, lapProfile);
        if (started) {
            executor->activeModule = TASK_MODULE_LAP;
        }
    } else if (selected == TASK_ID_4) {
        LineTracking_setEnabled(executor->config.lineTracking, false);
        LineTracking_reset(executor->config.lineTracking);
        started = StraightTaskControl_start(
            &executor->straightTask, &gStraightTask4Profile);
        if (started) {
            executor->activeModule = TASK_MODULE_STRAIGHT;
        }
    } else if (selected == TASK_ID_3) {
        LineTracking_setEnabled(executor->config.lineTracking, false);
        LineTracking_reset(executor->config.lineTracking);
        CarControl_stop(executor->config.car);
        executor->timerTaskRunning = true;
        executor->activeModule = TASK_MODULE_TIMER;
        started = true;
        TaskExecutor_log(executor,
            "TASK3_TIMER_STARTED (PA29/QS AGAIN TO STOP)\r\n");
    } else {
        TaskExecutor_log(executor,
            "TASK_UNSUPPORTED (TASK1 NEEDS ROUTE DEFINITION)\r\n");
    }

    if (started) {
        /*
         * 新任务可能直接从上一轮 DONE/FAULT 界面重启。
         * 先完整显示一次任务号清除旧页面，再进入局部秒表刷新。
         */
        if (TaskExecutor_oledAvailable(executor)) {
            *executor->config.oledReady =
                OLED_ShowTask((uint8_t) executor->activeTask);
        }
        if (TaskExecutor_oledAvailable(executor)) {
            *executor->config.oledReady = OLED_ShowStopwatch(
                (uint8_t) executor->activeTask, 0U);
        }
        /*
         * OLED 初始界面完成后再启动秒表，避免把显示传输时间计入比赛任务。
         */
        Stopwatch_start();
        executor->lastOledRefreshMs = SystemTime_getMs();
        TaskExecutor_log(executor, "TASK_TIMER_STARTED\r\n");
    }
    return started;
}

void TaskExecutor_update10ms(TaskExecutor *executor)
{
    bool wasRunning = TaskExecutor_isRunning(executor);

    if (executor->activeModule == TASK_MODULE_LAP) {
        LapTaskControl_update(&executor->lapTask);
    } else if (executor->activeModule == TASK_MODULE_STRAIGHT) {
        StraightTaskControl_update(&executor->straightTask);
    }

    if (wasRunning && !TaskExecutor_isRunning(executor)) {
        Stopwatch_stop();
        if (executor->activeModule == TASK_MODULE_LAP) {
            executor->lastTaskFault =
                executor->lapTask.state == LAP_TASK_FAULT;
        } else if (executor->activeModule == TASK_MODULE_STRAIGHT) {
            executor->lastTaskFault =
                executor->straightTask.state == STRAIGHT_TASK_FAULT;
        }
        TaskExecutor_logElapsed(executor);
        TaskExecutor_showResult(executor,
            executor->lastTaskFault ? OLED_TASK_RESULT_FAULT
                                    : OLED_TASK_RESULT_DONE);
    } else if (TaskExecutor_isRunning(executor) &&
        ((SystemTime_getMs() - executor->lastOledRefreshMs) >=
            TASK_OLED_REFRESH_MS)) {
        executor->lastOledRefreshMs = SystemTime_getMs();
        TaskExecutor_showStopwatch(executor);
    }
}

bool TaskExecutor_isRunning(const TaskExecutor *executor)
{
    if (executor->activeModule == TASK_MODULE_LAP) {
        return LapTaskControl_isRunning(&executor->lapTask);
    }
    if (executor->activeModule == TASK_MODULE_STRAIGHT) {
        return StraightTaskControl_isRunning(&executor->straightTask);
    }
    if (executor->activeModule == TASK_MODULE_TIMER) {
        return executor->timerTaskRunning;
    }
    return false;
}

TaskId TaskExecutor_getActiveTask(const TaskExecutor *executor)
{
    return executor->activeTask;
}

bool TaskExecutor_lastTaskFault(const TaskExecutor *executor)
{
    return executor->lastTaskFault;
}
