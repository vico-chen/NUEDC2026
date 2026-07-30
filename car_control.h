#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "motor_control.h"

/* 四轮车体支持的运动方向。 */
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

/* 车辆配置：四个电机、各轮正方向符号和默认速度。 */
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

/* 车辆当前速度和转弯内侧轮比例。 */
typedef struct {
    CarControl_Config config;
    CarControl_Motion motion;
    int16_t speedRpm;
    uint8_t turnInnerPercent;
} CarControl;

/* 初始化车体。 */
void CarControl_init(CarControl *car, const CarControl_Config *config);
/* 紧急停车：立即关闭 PWM，并清除每个车轮的 PID 状态。 */
void CarControl_emergencyStop(CarControl *car);
/* 主动停车与自由滑行。 */
void CarControl_stop(CarControl *car);
void CarControl_coast(CarControl *car);
/* 将车辆动作换算为四个车轮的带符号目标转速。 */
void CarControl_setMotion(CarControl *car, CarControl_Motion motion,
    int16_t speedRpm, uint8_t turnInnerPercent);
/* 读取当前车辆级速度和内侧轮比例。 */
int16_t CarControl_getSpeedRpm(const CarControl *car);
uint8_t CarControl_getTurnInnerPercent(const CarControl *car);
CarControl_Motion CarControl_getMotion(const CarControl *car);

#endif
