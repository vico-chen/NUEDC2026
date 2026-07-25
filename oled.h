#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

bool OLED_Init(void);
bool OLED_ShowTask(uint8_t taskNumber);
bool OLED_ShowTask1Endpoint(uint8_t endpointNumber);
bool OLED_ShowCalibration(uint8_t secondsRemaining);

#endif /* OLED_H_ */
