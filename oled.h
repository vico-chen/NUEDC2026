#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

/* 初始化 SSD1306 OLED；I2C 通信失败时返回 false。 */
bool OLED_Init(void);
/* 显示任务、端点、视觉数字、转向方向和标定倒计时。 */
bool OLED_ShowTask(uint8_t taskNumber);
bool OLED_ShowTask1Endpoint(uint8_t endpointNumber);
bool OLED_ShowTask2Endpoint(uint8_t endpointNumber);
bool OLED_ShowTask2Number(uint8_t number);
bool OLED_ShowTask2Turn(char direction);
bool OLED_ShowTask3Number(uint8_t number);
bool OLED_ShowTask3Turn(char direction);
/* endpoint=0 显示 AUTO，endpoint=1~4 显示对应端点。 */
bool OLED_ShowTask3Endpoint(uint8_t endpointNumber);
bool OLED_ShowCalibration(uint8_t secondsRemaining);
/* 以 3 倍大字显示整数秒，例如“12S”。 */
bool OLED_ShowStopwatch(uint64_t stopwatchMilliseconds);

#endif /* OLED_H_ */
