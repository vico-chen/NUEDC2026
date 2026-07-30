/*
 * 八路数字灰度传感器驱动（CD4051 多路选择器）
 *
 * AD0/AD1/AD2 选择 X1～X8，OUT 返回所选通道的数字电平。
 * 物理引脚由 SysConfig 生成，并通过 board_pins.h 适配。
 */
#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include <stdint.h>

#define GRAYSCALE_SENSOR_CHANNELS (8U)

/* 初始化地址线，并把 OUT 强制配置为上拉、施密特输入。 */
void Grayscale_Sensor_Init(void);
/* 按 X1～X8 顺序读取全部通道。 */
void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS]);
/* 读取指定通道，channel 范围为 0～7。 */
uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H_ */
