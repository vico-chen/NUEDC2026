#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_MANAGER_TASK_1 = 1,
    TASK_MANAGER_TASK_2 = 2,
    TASK_MANAGER_TASK_3 = 3
} TaskManager_Task;

typedef enum {
    TASK_MANAGER_TASK1_ENDPOINT_1 = 1,
    TASK_MANAGER_TASK1_ENDPOINT_2 = 2
} TaskManager_Task1Endpoint;

void TaskManager_init(bool oledReady);
void TaskManager_update(void);
TaskManager_Task TaskManager_getActiveTask(void);
TaskManager_Task1Endpoint TaskManager_getTask1Endpoint(void);
/* Returns true exactly once for each debounced, low-active PIN_STATUS press. */
bool TaskManager_takeStatusPressed(void);

#endif /* TASK_MANAGER_H_ */
