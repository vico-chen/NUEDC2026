#include "task_manager.h"

#include "oled.h"
#include "ti_msp_dl_config.h"

#define TASK_MANAGER_DEBOUNCE_SAMPLES (3U)

typedef struct {
    bool rawPressed;
    bool stablePressed;
    uint8_t debounceCount;
} TaskManager_ButtonState;

static TaskManager_Task gActiveTask;
static TaskManager_Task1Endpoint gTask1Endpoint;
static bool gOledReady;
static TaskManager_ButtonState gTaskButton;
static TaskManager_ButtonState gTask1EndpointButton;
static TaskManager_ButtonState gStatusButton;
static bool gStatusPressPending;

static bool TaskManager_readButtonPressed(void)
{
    /* PB21 is configured with pull-up: press connects the input to GND. */
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK_PORT,
        GPIO_BTN_PIN_TASK_PIN) == 0U);
}

static bool TaskManager_readTask1EndpointButtonPressed(void)
{
    /* S1 connects PA18 to 3.3 V when pressed; R14 pulls it down when idle. */
    return (DL_GPIO_readPins(GPIO_BTN_PIN_TASK1_CHANGE_PORT,
        GPIO_BTN_PIN_TASK1_CHANGE_PIN) != 0U);
}

static bool TaskManager_readStatusButtonPressed(void)
{
    /* PB1 is low-active: the load-complete switch connects it to GND. */
    return (DL_GPIO_readPins(GPIO_BTN_PIN_STATUS_PORT,
        GPIO_BTN_PIN_STATUS_PIN) == 0U);
}

/* Returns true once for each debounced press, never while the key is held. */
static bool TaskManager_updateButton(
    TaskManager_ButtonState *button, bool rawPressed)
{
    if (rawPressed != button->rawPressed) {
        button->rawPressed = rawPressed;
        button->debounceCount = 0U;
        return false;
    }

    if (button->debounceCount < TASK_MANAGER_DEBOUNCE_SAMPLES) {
        button->debounceCount++;
        return false;
    }

    if (button->stablePressed == button->rawPressed) {
        return false;
    }

    button->stablePressed = button->rawPressed;
    return button->stablePressed;
}

static void TaskManager_showActiveTask(void)
{
    if (gOledReady) {
        if (gActiveTask == TASK_MANAGER_TASK_1) {
            gOledReady = OLED_ShowTask1Endpoint((uint8_t) gTask1Endpoint);
        } else {
            gOledReady = OLED_ShowTask((uint8_t) gActiveTask);
        }
    }
}

static void TaskManager_advanceTask(void)
{
    if (gActiveTask == TASK_MANAGER_TASK_3) {
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

    if (gActiveTask == TASK_MANAGER_TASK_1) {
        TaskManager_showActiveTask();
    }
}

void TaskManager_init(bool oledReady)
{
    /* PB1/PB21 are low-active; PA18 is high-active on the LaunchPad. */
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_STATUS_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_TASK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_TASK1_CHANGE_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    gOledReady = oledReady;
    gActiveTask = TASK_MANAGER_TASK_1;
    gTask1Endpoint = TASK_MANAGER_TASK1_ENDPOINT_1;
    gTaskButton.rawPressed = TaskManager_readButtonPressed();
    gTaskButton.stablePressed = gTaskButton.rawPressed;
    gTaskButton.debounceCount = 0U;
    gTask1EndpointButton.rawPressed =
        TaskManager_readTask1EndpointButtonPressed();
    gTask1EndpointButton.stablePressed = gTask1EndpointButton.rawPressed;
    gTask1EndpointButton.debounceCount = 0U;
    gStatusButton.rawPressed = TaskManager_readStatusButtonPressed();
    gStatusButton.stablePressed = gStatusButton.rawPressed;
    gStatusButton.debounceCount = 0U;
    gStatusPressPending = false;
    TaskManager_showActiveTask();
}

void TaskManager_update(void)
{
    if (TaskManager_updateButton(
            &gTaskButton, TaskManager_readButtonPressed())) {
        TaskManager_advanceTask();
    }

    if (TaskManager_updateButton(&gTask1EndpointButton,
            TaskManager_readTask1EndpointButtonPressed())) {
        TaskManager_toggleTask1Endpoint();
    }

    if (TaskManager_updateButton(
            &gStatusButton, TaskManager_readStatusButtonPressed())) {
        gStatusPressPending = true;
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

bool TaskManager_takeStatusPressed(void)
{
    bool pressed = gStatusPressPending;

    gStatusPressPending = false;
    return pressed;
}
