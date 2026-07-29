/*
 * MPU6050 relative Z-axis angle driver.
 *
 * The angle is obtained by integrating gyro Z. It is a relative turn angle,
 * not an absolute compass heading, and must be reset at a known direction.
 */
#ifndef MPU6050_ANGLE_H_
#define MPU6050_ANGLE_H_

#include <stdbool.h>
#include <stdint.h>

typedef void (*MPU6050_CalibrationProgressCallback)(uint8_t secondsRemaining);

/* 初始化 MPU6050 并完成静止零偏标定。 */
bool MPU6050_Angle_init(void);
/* 带进度回调的初始化版本，可用于 OLED 倒计时。 */
bool MPU6050_Angle_initWithProgress(
    MPU6050_CalibrationProgressCallback progressCallback);
/* 周期读取 Z 轴角速度并积分航向角。 */
bool MPU6050_Angle_update(void);
/* 清零累计角度，不重新执行硬件初始化。 */
void MPU6050_Angle_reset(void);
/* 获取累计角度、当前角速度和 WHO_AM_I 设备号。 */
float MPU6050_Angle_getZDegrees(void);
float MPU6050_Angle_getZRateDps(void);
unsigned char MPU6050_Angle_getDeviceId(void);

#endif /* MPU6050_ANGLE_H_ */
