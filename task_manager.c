#include "task_manager.h"

#include "board_pins.h"

#define TASK_BUTTON_DEBOUNCE_SAMPLES (3U)

typedef struct {
    bool rawPressed;
    bool stablePressed;
    uint8_t debounceCount;
} TaskButtonState;

static TaskId gSelectedTask = TASK_ID_2;
static TaskButtonState gStartButton;
static TaskButtonState gChangeButton;
static bool gStartPending;
static bool gChangePending;

static bool TaskManager_readStartPressed(void)
{
    return DL_GPIO_readPins(
        BOARD_TASK_START_PORT, BOARD_TASK_START_PIN) == 0U;
}

static bool TaskManager_readChangePressed(void)
{
    return DL_GPIO_readPins(
        BOARD_TASK_CHANGE_PORT, BOARD_TASK_CHANGE_PIN) == 0U;
}

static void TaskManager_initButton(
    TaskButtonState *button, bool pressed)
{
    button->rawPressed = pressed;
    button->stablePressed = pressed;
    button->debounceCount = 0U;
}

/*
 * 连续读取到相同电平 30 ms 后才确认状态变化。
 * 仅在“松开 -> 按下”的稳定边沿返回 true，长按不会重复触发。
 */
static bool TaskManager_updateButton(
    TaskButtonState *button, bool rawPressed)
{
    if (rawPressed != button->rawPressed) {
        button->rawPressed = rawPressed;
        button->debounceCount = 0U;
        return false;
    }
    if (button->debounceCount < TASK_BUTTON_DEBOUNCE_SAMPLES) {
        button->debounceCount++;
        return false;
    }
    if (button->stablePressed == button->rawPressed) {
        return false;
    }

    button->stablePressed = button->rawPressed;
    return button->stablePressed;
}

void TaskManager_init(void)
{
    /*
     * SysConfig 负责物理引脚，任务层只依赖 BOARD_* 适配名。
     * PA29 和 PA13 都使用内部上拉，因此按下时读取到低电平。
     */
    DL_GPIO_initDigitalInputFeatures(BOARD_TASK_START_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_TASK_CHANGE_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(
        BOARD_TASK_START_PORT, BOARD_TASK_START_PIN);
    DL_GPIO_disableOutput(
        BOARD_TASK_CHANGE_PORT, BOARD_TASK_CHANGE_PIN);

    TaskManager_initButton(
        &gStartButton, TaskManager_readStartPressed());
    TaskManager_initButton(
        &gChangeButton, TaskManager_readChangePressed());
    gStartPending = false;
    gChangePending = false;
    gSelectedTask = TASK_ID_2;
}

void TaskManager_update10ms(void)
{
    if (TaskManager_updateButton(
            &gChangeButton, TaskManager_readChangePressed())) {
        gChangePending = true;
    }
    if (TaskManager_updateButton(
            &gStartButton, TaskManager_readStartPressed())) {
        gStartPending = true;
    }
}

bool TaskManager_takeStartPressed(void)
{
    bool result = gStartPending;
    gStartPending = false;
    return result;
}

bool TaskManager_takeChangePressed(void)
{
    bool result = gChangePending;
    gChangePending = false;
    return result;
}

bool TaskManager_select(TaskId task)
{
    if ((task < TASK_ID_1) || (task > TASK_ID_6)) {
        return false;
    }
    gSelectedTask = task;
    return true;
}

/*
 * 当前可选择任务为 2、3、4、5、6，其中任务 3 仅用于人工计时。
 * 按键循环只跳过尚未定义路线的任务 1。
 */
TaskId TaskManager_selectNextSupported(void)
{
    switch (gSelectedTask) {
        case TASK_ID_2:
            gSelectedTask = TASK_ID_3;
            break;
        case TASK_ID_3:
            gSelectedTask = TASK_ID_4;
            break;
        case TASK_ID_4:
            gSelectedTask = TASK_ID_5;
            break;
        case TASK_ID_5:
            gSelectedTask = TASK_ID_6;
            break;
        default:
            gSelectedTask = TASK_ID_2;
            break;
    }
    return gSelectedTask;
}

TaskId TaskManager_getSelected(void)
{
    return gSelectedTask;
}
