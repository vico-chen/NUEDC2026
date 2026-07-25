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

bool MPU6050_Angle_init(void);
bool MPU6050_Angle_initWithProgress(
    MPU6050_CalibrationProgressCallback progressCallback);
bool MPU6050_Angle_update(void);
void MPU6050_Angle_reset(void);
float MPU6050_Angle_getZDegrees(void);
float MPU6050_Angle_getZRateDps(void);
unsigned char MPU6050_Angle_getDeviceId(void);

#endif /* MPU6050_ANGLE_H_ */
