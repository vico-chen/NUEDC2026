#include "task_executor.h"

#include "oled.h"
#include "stopwatch.h"
#include "task_profiles.h"

/* 统一通过回调输出任务日志，避免任务模块直接依赖某个 UART。 */
static void TaskExecutor_log(
    const TaskExecutor *executor, const char *text)
{
    if (executor->config.log != 0) {
        executor->config.log(text);
    }
}

/* 根据按键选择的任务号取得对应的环形巡线参数。 */
static const LapTask_Profile *TaskExecutor_getLapProfile(
    TaskManager_Task task)
{
    switch (task) {
        /* 当前版本仅将 Task2、5、6 映射到环形巡线模块。 */
        case TASK_MANAGER_TASK_2:
            return &gLapTask2Profile;
        case TASK_MANAGER_TASK_5:
            return &gLapTask5Profile;
        case TASK_MANAGER_TASK_6:
            return &gLapTask6Profile;
        default:
            return 0;
    }
}

/* 同时复位两个子模块，确保任何任务切换都会先解除电机控制。 */
static void TaskExecutor_stopActiveModule(TaskExecutor *executor)
{
    LapTaskControl_reset(&executor->lapTask);
    StraightTaskControl_reset(&executor->straightTask);
    executor->activeModule = TASK_EXECUTOR_MODULE_NONE;
    // Stopwatch_stop();
}

static bool TaskExecutor_startSelected(TaskExecutor *executor)
{
    const LapTask_Profile *lapProfile =
        TaskExecutor_getLapProfile(executor->selectedTask);
    bool started = false;

    /* 启动新任务前清除上一次任务残留的状态和编码器累计值。 */
    LapTaskControl_reset(&executor->lapTask);
    StraightTaskControl_reset(&executor->straightTask);

    if (lapProfile != 0) {
        /* Task2、5、6 共用同一个环形状态机，只传入不同参数表。 */
        started = LapTaskControl_start(&executor->lapTask,
            (uint8_t) executor->selectedTask, lapProfile);
        if (started) {
            executor->activeModule = TASK_EXECUTOR_MODULE_LAP;
        }
    } else if (executor->selectedTask == TASK_MANAGER_TASK_4) {
        /*
         * Task4 是编码器定距直行，不使用八路灰度巡线。
         * 启动前必须关闭巡线控制，防止两个模块同时写入车辆速度。
         */
        LineTracking_setEnabled(executor->config.lineTracking, false);
        LineTracking_setDebugEnabled(
            executor->config.lineTracking, false);
        LineTracking_reset(executor->config.lineTracking);
        started = StraightTaskControl_start(
            &executor->straightTask, &gStraightTask4Profile);
        if (started) {
            executor->activeModule =
                TASK_EXECUTOR_MODULE_STRAIGHT;
        }
    } else {
        /* 未配置执行模块的任务只提示，不允许车辆动作。 */
        TaskExecutor_log(executor,
            "TASK3 HAS NO CONTROL MODULE\r\n");
    }

    if (started) {
        /* 只有模块启动成功后才启动秒表。 */
        Stopwatch_start();
        executor->lastStopwatchDisplayMs = systick_ms;
    } else {
        executor->activeModule = TASK_EXECUTOR_MODULE_NONE;
    }
    return started;
}

void TaskExecutor_init(TaskExecutor *executor,
    const TaskExecutor_Config *config)
{
    /* 从公共配置中分别构造两个子模块所需的最小依赖集合。 */
    const LapTask_Config lapConfig = {
        .car = config->car,
        .lineTracking = config->lineTracking,
        .motors = {
            config->motors[0], config->motors[1],
            config->motors[2], config->motors[3]
        },
        .log = config->log,
    };
    const StraightTask_Config straightConfig = {
        .car = config->car,
        .motors = {
            config->motors[0], config->motors[1],
            config->motors[2], config->motors[3]
        },
        .log = config->log,
    };

    executor->config = *config;
    LapTaskControl_init(&executor->lapTask, &lapConfig);
    StraightTaskControl_init(
        &executor->straightTask, &straightConfig);
    executor->activeModule = TASK_EXECUTOR_MODULE_NONE;
    executor->selectedTask = TASK_MANAGER_TASK_1;
    executor->lastStopwatchDisplayMs = systick_ms;
}

void TaskExecutor_reset(TaskExecutor *executor)
{
    /* 用于串口急停或外部取消任务。 */
    TaskExecutor_stopActiveModule(executor);
}

bool TaskExecutor_isRunning(const TaskExecutor *executor)
{
    /* 根据当前模块查询它自己的运行状态。 */
    if (executor->activeModule == TASK_EXECUTOR_MODULE_LAP) {
        return LapTaskControl_isRunning(&executor->lapTask);
    }
    if (executor->activeModule ==
        TASK_EXECUTOR_MODULE_STRAIGHT) {
        return StraightTaskControl_isRunning(
            &executor->straightTask);
    }
    return false;
}

void TaskExecutor_update(TaskExecutor *executor,
    TaskManager_Task selectedTask, bool startPressed)
{
    bool wasRunning;

    /*
     * TASK 按键切换任务时，如果车辆仍在执行旧任务，
     * 先复位旧模块并停车，再记录新的任务号。
     */
    if (selectedTask != executor->selectedTask) {
        if (TaskExecutor_isRunning(executor)) {
            TaskExecutor_stopActiveModule(executor);
            TaskExecutor_log(executor,
                "TASK SELECTION CHANGED, STOPPED\r\n");
        }
        executor->selectedTask = selectedTask;
    }

    /* 启动按键只在当前没有任务运行时生效。 */
    if (startPressed) {
        if (TaskExecutor_isRunning(executor)) {
            TaskExecutor_log(executor, "TASK ALREADY RUNNING\r\n");
        } else {
            (void) TaskExecutor_startSelected(executor);
        }
    }

    /* 每个 10 ms 周期只推进当前选中的一个状态机。 */
    wasRunning = TaskExecutor_isRunning(executor);
    if (executor->activeModule == TASK_EXECUTOR_MODULE_LAP) {
        LapTaskControl_update(&executor->lapTask);
    } else if (executor->activeModule ==
        TASK_EXECUTOR_MODULE_STRAIGHT) {
        StraightTaskControl_update(&executor->straightTask);
    }

    /* 任务运行期间每 500 ms 把当前用时刷新到 OLED。 */
    if (TaskExecutor_isRunning(executor) &&
        ((systick_ms - executor->lastStopwatchDisplayMs) >= 500U)) {
        if (executor->config.oledReady) {
            executor->config.oledReady =
                OLED_ShowStopwatch(Stopwatch_getCurrentMs());
        }
        executor->lastStopwatchDisplayMs = systick_ms;
    }
}
