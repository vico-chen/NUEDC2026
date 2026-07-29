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

/* Task2：AUTO 使用 OpenMV，END1/END2 在第二个十字路口转向。 */
typedef enum {
    TASK_MANAGER_TASK2_ENDPOINT_AUTO = 0,
    TASK_MANAGER_TASK2_ENDPOINT_1 = 1, /* 第二个十字路口左转 */
    TASK_MANAGER_TASK2_ENDPOINT_2 = 2  /* 第二个十字路口右转 */
} TaskManager_Task2Endpoint;

/*
 * Task3 端点预设：
 * AUTO 保留纯 OpenMV 路线；END1~END4 对应两次路口转向组合。
 */
typedef enum {
    TASK_MANAGER_TASK3_ENDPOINT_AUTO = 0,
    TASK_MANAGER_TASK3_ENDPOINT_1 = 1, /* 左、左 */
    TASK_MANAGER_TASK3_ENDPOINT_2 = 2, /* 左、右 */
    TASK_MANAGER_TASK3_ENDPOINT_3 = 3, /* 右、左 */
    TASK_MANAGER_TASK3_ENDPOINT_4 = 4  /* 右、右 */
} TaskManager_Task3Endpoint;

void TaskManager_init(bool oledReady);
void TaskManager_update(void);
TaskManager_Task TaskManager_getActiveTask(void);
TaskManager_Task1Endpoint TaskManager_getTask1Endpoint(void);
TaskManager_Task2Endpoint TaskManager_getTask2Endpoint(void);
TaskManager_Task3Endpoint TaskManager_getTask3Endpoint(void);
/* Returns true exactly once for each debounced, low-active PIN_STATUS press. */
bool TaskManager_takeStatusPressed(void);
/* Returns true exactly once for each debounced PIN_STATUS release. */
bool TaskManager_takeStatusReleased(void);

#endif /* TASK_MANAGER_H_ */
