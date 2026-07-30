#include "task_manager.h"

#include "oled.h"
#include "ti_msp_dl_config.h"

/* 按键电平连续稳定 3 个 10 ms 周期后才确认状态变化。 */
#define TASK_MANAGER_DEBOUNCE_SAMPLES (3U)

/* 单个按键的软件消抖状态。 */
typedef struct {
    bool rawPressed;
    bool stablePressed;
    uint8_t debounceCount;
} TaskManager_ButtonState;

static bool gOledReady;
static TaskManager_ButtonState gTaskButton;
static TaskManager_ButtonState gTaskStartButton;
static TaskManager_ButtonState gStatusButton;
static TaskManager_Task gActiveTask;
static bool gTaskStartPressPending;
static bool gStatusPressPending;
static bool gStatusReleasePending;

/* TASK 选择键使用上拉输入，因此按下时读取为低电平。 */
static bool TaskManager_readTaskButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK_PORT,
        GPIO_BTN_PIN_TASK_PIN) == 0U);
}

/* TASK_START 使用下拉输入，因此按下时读取为高电平。 */
static bool TaskManager_readTaskStartButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK_START_PORT,
        GPIO_BTN_PIN_TASK_START_PIN) != 0U);
}

/* STATUS 使用上拉输入，因此按下时读取为低电平。 */
static bool TaskManager_readStatusButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_STATUS_PORT,
        GPIO_BTN_PIN_STATUS_PIN) == 0U);
}

/*
 * 更新一个按键的消抖状态。
 * 返回 1 表示确认按下，-1 表示确认释放，0 表示没有新事件。
 */
static int8_t TaskManager_updateButton(
    TaskManager_ButtonState *button, bool rawPressed)
{
    if (rawPressed != button->rawPressed) {
        button->rawPressed = rawPressed;
        button->debounceCount = 0U;
        return 0;
    }

    if (button->debounceCount < TASK_MANAGER_DEBOUNCE_SAMPLES) {
        button->debounceCount++;
        return 0;
    }

    if (button->stablePressed == button->rawPressed) {
        return 0;
    }

    button->stablePressed = button->rawPressed;
    return button->stablePressed ? 1 : -1;
}

/* 用当前真实电平初始化按键，避免上电时产生一次虚假按键事件。 */
static void TaskManager_initButton(
    TaskManager_ButtonState *button, bool rawPressed)
{
    button->rawPressed = rawPressed;
    button->stablePressed = rawPressed;
    button->debounceCount = 0U;
}

/* 在 OLED 上显示当前选中的任务号。 */
static void TaskManager_showActiveTask(void)
{
    if (gOledReady) {
        gOledReady = OLED_ShowTask((uint8_t) gActiveTask);
    }
}

/* TASK 键每按一次任务号加一，Task6 后回到 Task1。 */
static void TaskManager_advanceTask(void)
{
    if (gActiveTask == TASK_MANAGER_TASK_6) {
        gActiveTask = TASK_MANAGER_TASK_1;
    } else {
        gActiveTask = (TaskManager_Task) ((uint8_t) gActiveTask + 1U);
    }
    TaskManager_showActiveTask();
}

void TaskManager_init(bool oledReady)
{
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_STATUS_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_TASK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_TASK_START_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    gOledReady = oledReady;
    gActiveTask = TASK_MANAGER_TASK_1;
    gTaskStartPressPending = false;
    gStatusPressPending = false;
    gStatusReleasePending = false;

    TaskManager_initButton(&gTaskButton, TaskManager_readTaskButtonPressed());
    TaskManager_initButton(&gTaskStartButton,
        TaskManager_readTaskStartButtonPressed());
    TaskManager_initButton(&gStatusButton,
        TaskManager_readStatusButtonPressed());

    TaskManager_showActiveTask();
}

void TaskManager_update(void)
{   
    /* 任务选择键按下后立即切换并刷新 OLED。 */
    if (TaskManager_updateButton(
            &gTaskButton, TaskManager_readTaskButtonPressed()) > 0) {
        TaskManager_advanceTask();
    }

    if (TaskManager_updateButton(
            &gTaskStartButton, TaskManager_readTaskStartButtonPressed()) > 0) {
        gTaskStartPressPending = true;
    }

    /* STATUS 同时保留按下和释放事件，供以后需要长按的任务使用。 */
    {
        int8_t statusEvent = TaskManager_updateButton(
            &gStatusButton, TaskManager_readStatusButtonPressed());

        if (statusEvent > 0) {
            gStatusPressPending = true;
        } else if (statusEvent < 0) {
            gStatusReleasePending = true;
        }
    }
}

TaskManager_Task TaskManager_getActiveTask(void)
{
    return gActiveTask;
}

bool TaskManager_taskStartPressed(void)
{
    bool pressed = gTaskStartPressPending;

    gTaskStartPressPending = false;
    return pressed;
}

bool TaskManager_takeStatusPressed(void)
{
    bool pressed = gStatusPressPending;

    gStatusPressPending = false;
    return pressed;
}

bool TaskManager_takeStatusReleased(void)
{
    bool released = gStatusReleasePending;

    gStatusReleasePending = false;
    return released;
}
    /* 启动键事件保存为一次性标志，等待主循环读取。 */
