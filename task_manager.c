#include "task_manager.h"

#include "oled.h"
#include "ti_msp_dl_config.h"

#define TASK_MANAGER_DEBOUNCE_SAMPLES (3U)

static TaskManager_Task gActiveTask;
static bool gOledReady;
static bool gRawPressed;
static bool gStablePressed;
static uint8_t gDebounceCount;

static bool TaskManager_readButtonPressed(void)
{
    /* PB21 is configured with pull-up: press connects the input to GND. */
    return (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_PIN_TASK_PIN) == 0U);
}

static void TaskManager_showActiveTask(void)
{
    if (gOledReady) {
        gOledReady = OLED_ShowTask((uint8_t) gActiveTask);
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

void TaskManager_init(bool oledReady)
{
    /* Also configure it here so the required pull-up is enforced at runtime. */
    DL_GPIO_initDigitalInputFeatures(GPIO_BTN_PIN_TASK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    gOledReady = oledReady;
    gActiveTask = TASK_MANAGER_TASK_1;
    gRawPressed = TaskManager_readButtonPressed();
    gStablePressed = gRawPressed;
    gDebounceCount = 0U;
    TaskManager_showActiveTask();
}

void TaskManager_update(void)
{
    bool rawPressed = TaskManager_readButtonPressed();

    if (rawPressed != gRawPressed) {
        gRawPressed = rawPressed;
        gDebounceCount = 0U;
        return;
    }

    if (gDebounceCount < TASK_MANAGER_DEBOUNCE_SAMPLES) {
        gDebounceCount++;
        return;
    }

    if (gStablePressed == gRawPressed) {
        return;
    }

    gStablePressed = gRawPressed;
    if (gStablePressed) {
        TaskManager_advanceTask();
    }
}

TaskManager_Task TaskManager_getActiveTask(void)
{
    return gActiveTask;
}
