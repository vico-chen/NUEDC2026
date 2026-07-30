#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "motor_control.h"

/*
 * 四轮底盘运动分配模块
 *
 * 将前进、后退、原地转向和弧线转向转换为四个电机的目标 RPM。
 * 车轮布局：C 左前、B 右前、D 左后、A 右后。
 */

/* 底盘支持的八种基础运动模式。 */
typedef enum {
    CAR_CONTROL_FORWARD,
    CAR_CONTROL_BACKWARD,
    CAR_CONTROL_PIVOT_LEFT,
    CAR_CONTROL_PIVOT_RIGHT,
    CAR_CONTROL_FORWARD_LEFT,
    CAR_CONTROL_FORWARD_RIGHT,
    CAR_CONTROL_BACKWARD_LEFT,
    CAR_CONTROL_BACKWARD_RIGHT
} CarControl_Motion;

/* 四轮对象、安装方向和底盘速度限值。 */
typedef struct {
    MotorControl *rightRearMotor;
    MotorControl *rightFrontMotor;
    MotorControl *leftFrontMotor;
    MotorControl *leftRearMotor;
    int8_t rightRearForwardSign;
    int8_t rightFrontForwardSign;
    int8_t leftFrontForwardSign;
    int8_t leftRearForwardSign;
    int16_t maximumSpeedRpm;
    int16_t defaultSpeedRpm;
    uint8_t defaultTurnInnerPercent;
} CarControl_Config;

/* 保存最近一次底盘速度与弧线内轮百分比。 */
typedef struct {
    CarControl_Config config;
    int16_t speedRpm;
    uint8_t turnInnerPercent;
} CarControl;

/* 绑定四个电机并加载默认参数。 */
void CarControl_init(CarControl *car, const CarControl_Config *config);
/* 紧急停车：立即关闭四轮驱动并清空 PID。 */
void CarControl_emergencyStop(CarControl *car);
/* 正常停车：目标设为 0，由速度环主动制动。 */
void CarControl_stop(CarControl *car);
/* 四轮立即滑行，用于定角转向的惯性阶段。 */
void CarControl_coast(CarControl *car);
/* 分配四轮目标速度；弧线模式由 innerPercent 缩放内轮。 */
void CarControl_setMotion(CarControl *car, CarControl_Motion motion,
    int16_t speedRpm, uint8_t turnInnerPercent);
/* 读取最近一次底盘基准速度。 */
int16_t CarControl_getSpeedRpm(const CarControl *car);
/* 读取最近一次弧线内轮速度百分比。 */
uint8_t CarControl_getTurnInnerPercent(const CarControl *car);

#endif
