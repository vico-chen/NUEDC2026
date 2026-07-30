#ifndef TASK_PROFILES_H_
#define TASK_PROFILES_H_

#include "lap_task_control.h"
#include "straight_task_control.h"

/* 各任务独立参数表；修改速度和过程参数时只需编辑 task_profiles.c。 */
extern const LapTask_Profile gLapTask1Profile;
extern const LapTask_Profile gLapTask2Profile;
extern const LapTask_Profile gLapTask5Profile;
extern const LapTask_Profile gLapTask6Profile;
/* Task4 使用编码器定距直行参数。 */
extern const StraightTask_Profile gStraightTask4Profile;

#endif /* TASK_PROFILES_H_ */
