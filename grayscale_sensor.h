/*
 * 八路数字灰度模块（CD4051 多路选择器）。
 * SysConfig 引脚名称为 AD0、AD1、AD2 和 OUT。
 */
#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include <stdint.h>

#define GRAYSCALE_SENSOR_CHANNELS (8U)

/* 初始化通道选择线和数字输出输入端。 */
void Grayscale_Sensor_Init(void);
/* 按 X1～X8 顺序读取全部通道；返回值为对应通道的数字电平。 */
void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS]);
/* 单独读取指定通道（0～7）；通道越界时返回 0。 */
uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H_ */
