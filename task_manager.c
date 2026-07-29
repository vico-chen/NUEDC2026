#include "task_manager.h"

#include "oled.h"
#include "ti_msp_dl_config.h"

#define TASK_MANAGER_DEBOUNCE_SAMPLES (3U)

typedef struct {
    bool rawPressed;
    bool stablePressed;
    uint8_t debounceCount;
} TaskManager_ButtonState;

static bool gOledReady;
static TaskManager_ButtonState gTaskButton;
static TaskManager_ButtonState gTaskStartButton;
static TaskManager_ButtonState gStatusButton;
static TaskManager_Task1Endpoint gTask1Endpoint;
static TaskManager_Task gActiveTask;
static bool gTaskStartPressPending;
static bool gStatusPressPending;
static bool gStatusReleasePending;

static bool TaskManager_readTaskButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK_PORT,
        GPIO_BTN_PIN_TASK_PIN) == 0U);
}

static bool TaskManager_readTaskStartButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK_START_PORT,
        GPIO_BTN_PIN_TASK_START_PIN) != 0U);
}

static bool TaskManager_readStatusButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BTN_PIN_STATUS_PORT,
        GPIO_BTN_PIN_STATUS_PIN) == 0U);
}

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

static void TaskManager_initButton(
    TaskManager_ButtonState *button, bool rawPressed)
{
    button->rawPressed = rawPressed;
    button->stablePressed = rawPressed;
    button->debounceCount = 0U;
}

static void TaskManager_showTask1(void)
{
    if (gOledReady) {
        gOledReady = OLED_ShowTask1Endpoint((uint8_t) gTask1Endpoint);
    }
}

static void TaskManager_showActiveTask(void)
{
    if (gOledReady) {
        gOledReady = OLED_ShowTask((uint8_t) gActiveTask);
    }
}

static void TaskManager_advanceTask(void)
{
    if (gActiveTask == TASK_MANAGER_TASK_6) {
        gActiveTask = TASK_MANAGER_TASK_1;
    } else {
        gActiveTask = (TaskManager_Task) ((uint8_t) gActiveTask + 1U);
    }
    TaskManager_showActiveTask();
}

static void TaskManager_toggleTask1Endpoint(void)
{
    if (gTask1Endpoint == TASK_MANAGER_TASK1_ENDPOINT_1) {
        gTask1Endpoint = TASK_MANAGER_TASK1_ENDPOINT_2;
    } else {
        gTask1Endpoint = TASK_MANAGER_TASK1_ENDPOINT_1;
    }
    TaskManager_showTask1();
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
    gTask1Endpoint = TASK_MANAGER_TASK1_ENDPOINT_1;
    gActiveTask = TASK_MANAGER_TASK_1;
    gTaskStartPressPending = false;
    gStatusPressPending = false;
    gStatusReleasePending = false;

    TaskManager_initButton(&gTaskButton, TaskManager_readTaskButtonPressed());
    TaskManager_initButton(&gTaskStartButton,
        TaskManager_readTaskStartButtonPressed());
    TaskManager_initButton(&gStatusButton,
        TaskManager_readStatusButtonPressed());

    TaskManager_showTask1();
}

void TaskManager_update(void)
{   
    if (TaskManager_updateButton(
            &gTaskButton, TaskManager_readTaskButtonPressed()) > 0) {
        TaskManager_advanceTask();
    }

    if (TaskManager_updateButton(
            &gTaskStartButton, TaskManager_readTaskStartButtonPressed()) > 0) {
        gTaskStartPressPending = true;
    }

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

TaskManager_Task1Endpoint TaskManager_getTask1Endpoint(void)
{
    return gTask1Endpoint;
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
