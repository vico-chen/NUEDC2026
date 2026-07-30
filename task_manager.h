#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

/* TASK 选择按键可在 1～6 之间循环切换。 */
typedef enum {
    TASK_MANAGER_TASK_1 = 1,
    TASK_MANAGER_TASK_2 = 2,
    TASK_MANAGER_TASK_3 = 3,
    TASK_MANAGER_TASK_4 = 4,
    TASK_MANAGER_TASK_5 = 5,
    TASK_MANAGER_TASK_6 = 6
} TaskManager_Task;

/* 初始化三个按键和 OLED 任务显示。 */
void TaskManager_init(bool oledReady);
/* 每 10 ms 调用一次，完成按键消抖和事件生成。 */
void TaskManager_update(void);
/* 取得 TASK 按键当前选中的任务号。 */
TaskManager_Task TaskManager_getActiveTask(void);
/* 读取并清除一次任务启动按键事件。 */
bool TaskManager_taskStartPressed(void);
/* 状态按键事件接口，保留给后续任务使用。 */
bool TaskManager_takeStatusPressed(void);
bool TaskManager_takeStatusReleased(void);

#endif /* TASK_MANAGER_H_ */
