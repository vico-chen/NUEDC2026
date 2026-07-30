#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OLED_TASK_RESULT_DONE,
    OLED_TASK_RESULT_FAULT,
    OLED_TASK_RESULT_CANCELLED
} OLED_TaskResult;

/* 初始化 128×64 SSD1306；I2C 通信失败时返回 false。 */
bool OLED_Init(void);
/* 显示当前选择的任务号。 */
bool OLED_ShowTask(uint8_t taskNumber);
/* 显示任务号和大字运行秒表。 */
bool OLED_ShowStopwatch(uint8_t taskNumber, uint32_t elapsedMs);
/* 显示任务最终状态和冻结的任务用时。 */
bool OLED_ShowTaskResult(uint8_t taskNumber,
    OLED_TaskResult result, uint32_t elapsedMs);
/* 通用自检文本接口，保留对原 oled_test 模块的兼容。 */
bool OLED_ShowMessage(const char *text);

#endif /* OLED_H_ */
