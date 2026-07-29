/*
 * 8-ch digital grayscale (CD4051), based on vendor MSPM0 Grayscale_Read.
 * Pins come from SysConfig GPIO_GRAYSCALE: AD0/AD1/AD2/OUT.
 */
#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include <stdint.h>

#define GRAYSCALE_SENSOR_CHANNELS (8U)

/* 初始化八路灰度模块的通道选择线和数字输出。 */
void Grayscale_Sensor_Init(void);
/* 按 X1～X8 顺序读取全部通道，1 表示检测有效。 */
void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS]);
/* 单独读取指定通道；通道越界时返回 0。 */
uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H_ */
