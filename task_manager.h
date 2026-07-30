#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_ID_1 = 1,
    TASK_ID_2,
    TASK_ID_3,
    TASK_ID_4,
    TASK_ID_5,
    TASK_ID_6
} TaskId;

/* PA29 启动当前任务；PA13 请求切换任务。两个按键均在 10 ms 节拍中消抖。 */
void TaskManager_init(void);
void TaskManager_update10ms(void);
bool TaskManager_takeStartPressed(void);
bool TaskManager_takeChangePressed(void);
bool TaskManager_select(TaskId task);
TaskId TaskManager_selectNextSupported(void);
TaskId TaskManager_getSelected(void);

#endif /* TASK_MANAGER_H_ */
