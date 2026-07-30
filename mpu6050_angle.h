/*
 * MPU6050 Z 轴相对转角驱动
 *
 * 使用陀螺仪 Z 轴角速度积分得到相对角度。它不是绝对航向，会随时间漂移；
 * 每次定角任务开始前需要在已知方向清零。
 */
#ifndef MPU6050_ANGLE_H_
#define MPU6050_ANGLE_H_

#include <stdbool.h>

/* 初始化传感器并在静止状态下采集 500 个样本校准零偏。 */
bool MPU6050_Angle_init(void);
/* 读取 Z 轴角速度并完成一次 10 ms 积分。 */
bool MPU6050_Angle_update(void);
/* 将当前方向设为相对 0°，不会重新校准零偏。 */
void MPU6050_Angle_reset(void);
/* 返回累计的 Z 轴相对角度，单位为度。 */
float MPU6050_Angle_getZDegrees(void);
/* 返回当前去零偏后的 Z 轴角速度，单位为度/秒。 */
float MPU6050_Angle_getZRateDps(void);
/* 返回 WHO_AM_I 寄存器值。 */
unsigned char MPU6050_Angle_getDeviceId(void);

#endif /* MPU6050_ANGLE_H_ */
