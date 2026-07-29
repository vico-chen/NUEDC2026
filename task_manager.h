#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_MANAGER_TASK_1 = 1,
    TASK_MANAGER_TASK_2 = 2,
    TASK_MANAGER_TASK_3 = 3,
    TASK_MANAGER_TASK_4 = 4,
    TASK_MANAGER_TASK_5 = 5,
    TASK_MANAGER_TASK_6 = 6
} TaskManager_Task;

typedef enum {
    TASK_MANAGER_TASK1_ENDPOINT_1 = 1,
    TASK_MANAGER_TASK1_ENDPOINT_2 = 2
} TaskManager_Task1Endpoint;

typedef enum {
    TASK_MANAGER_TASK2_ENDPOINT_AUTO = 0,
    TASK_MANAGER_TASK2_ENDPOINT_1 = 1,
    TASK_MANAGER_TASK2_ENDPOINT_2 = 2
} TaskManager_Task2Endpoint;

typedef enum {
    TASK_MANAGER_TASK3_ENDPOINT_AUTO = 0,
    TASK_MANAGER_TASK3_ENDPOINT_1 = 1,
    TASK_MANAGER_TASK3_ENDPOINT_2 = 2,
    TASK_MANAGER_TASK3_ENDPOINT_3 = 3,
    TASK_MANAGER_TASK3_ENDPOINT_4 = 4
} TaskManager_Task3Endpoint;

void TaskManager_init(bool oledReady);
void TaskManager_update(void);
TaskManager_Task TaskManager_getActiveTask(void);
TaskManager_Task1Endpoint TaskManager_getTask1Endpoint(void);
bool TaskManager_taskStartPressed(void);
bool TaskManager_takeStatusPressed(void);
bool TaskManager_takeStatusReleased(void);

#endif /* TASK_MANAGER_H_ */
