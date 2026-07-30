#include "car_control.h"

/*
 * 按“左后、左前、右前、右后”的底盘语义写入四轮目标。
 * forwardSign 用于吸收左右电机镜像安装造成的电机本体方向差异。
 */
static void CarControl_setWheelRpm(CarControl *car,
    int16_t leftRearRpm, int16_t leftFrontRpm,
    int16_t rightFrontRpm, int16_t rightRearRpm)
{
    MotorControl_setTargetRpm(car->config.rightRearMotor,
        (int16_t) (rightRearRpm * car->config.rightRearForwardSign));
    MotorControl_setTargetRpm(car->config.rightFrontMotor,
        (int16_t) (rightFrontRpm * car->config.rightFrontForwardSign));
    MotorControl_setTargetRpm(car->config.leftFrontMotor,
        (int16_t) (leftFrontRpm * car->config.leftFrontForwardSign));
    MotorControl_setTargetRpm(car->config.leftRearMotor,
        (int16_t) (leftRearRpm * car->config.leftRearForwardSign));
}

/* 保存配置和默认参数，随后用正常制动方式把四轮目标置为 0。 */
void CarControl_init(CarControl *car, const CarControl_Config *config)
{
    car->config = *config;
    car->speedRpm = config->defaultSpeedRpm;
    car->turnInnerPercent = config->defaultTurnInnerPercent;
    CarControl_stop(car);
}

/* 正常停车仍保留速度闭环，由每个电机主动制动到零速死区。 */
void CarControl_stop(CarControl *car)
{
    CarControl_setWheelRpm(car, 0, 0, 0, 0);
}

/*
 * 紧急命令必须立即撤销驱动。不能调用 CarControl_stop()，因为目标为 0
 * 会进入闭环主动制动；若编码器极性或信号异常，反而可能产生危险输出。
 */
void CarControl_emergencyStop(CarControl *car)
{
    CarControl_coast(car);
}

/* 逐个通知电机关闭 PWM 并清除 PID。 */
void CarControl_coast(CarControl *car)
{
    MotorControl_coast(car->config.rightRearMotor);
    MotorControl_coast(car->config.rightFrontMotor);
    MotorControl_coast(car->config.leftFrontMotor);
    MotorControl_coast(car->config.leftRearMotor);
}

/*
 * 底盘运动学分配：
 * - 直行：左右轮同速；
 * - 原地转向：左右轮反向同速；
 * - 弧线转向：外轮保持 speedRpm，内轮乘以 turnInnerPercent。
 */
void CarControl_setMotion(CarControl *car, CarControl_Motion motion,
    int16_t speedRpm, uint8_t turnInnerPercent)
{
    int16_t innerWheelRpm;

    /* 底盘方向由 motion 表示，因此速度参数统一使用绝对值。 */
    if (speedRpm < 0) {
        speedRpm = (int16_t) (-speedRpm);
    }
    if (speedRpm > car->config.maximumSpeedRpm) {
        speedRpm = car->config.maximumSpeedRpm;
    }
    if (turnInnerPercent > 100U) {
        turnInnerPercent = 100U;
    }

    car->speedRpm = speedRpm;
    car->turnInnerPercent = turnInnerPercent;
    /* 使用 32 位中间值，避免 16 位乘法溢出。 */
    innerWheelRpm = (int16_t) (((int32_t) speedRpm *
        turnInnerPercent) / 100);

    switch (motion) {
        case CAR_CONTROL_FORWARD:
            /* 四轮同向前进。 */
            CarControl_setWheelRpm(
                car, speedRpm, speedRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_BACKWARD:
            /* 四轮同向后退。 */
            CarControl_setWheelRpm(
                car, -speedRpm, -speedRpm, -speedRpm, -speedRpm);
            break;
        case CAR_CONTROL_PIVOT_LEFT:
            /* 左轮后退、右轮前进，绕车体中心左转。 */
            CarControl_setWheelRpm(
                car, -speedRpm, -speedRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_PIVOT_RIGHT:
            /* 左轮前进、右轮后退，绕车体中心右转。 */
            CarControl_setWheelRpm(
                car, speedRpm, speedRpm, -speedRpm, -speedRpm);
            break;
        case CAR_CONTROL_FORWARD_LEFT:
            /* 左侧为内轮。 */
            CarControl_setWheelRpm(
                car, innerWheelRpm, innerWheelRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_FORWARD_RIGHT:
            /* 右侧为内轮。 */
            CarControl_setWheelRpm(
                car, speedRpm, speedRpm, innerWheelRpm, innerWheelRpm);
            break;
        case CAR_CONTROL_BACKWARD_LEFT:
            CarControl_setWheelRpm(car, -innerWheelRpm, -innerWheelRpm,
                -speedRpm, -speedRpm);
            break;
        case CAR_CONTROL_BACKWARD_RIGHT:
            CarControl_setWheelRpm(car, -speedRpm, -speedRpm,
                -innerWheelRpm, -innerWheelRpm);
            break;
        default:
            CarControl_stop(car);
            break;
    }
}

int16_t CarControl_getSpeedRpm(const CarControl *car)
{
    return car->speedRpm;
}

uint8_t CarControl_getTurnInnerPercent(const CarControl *car)
{
    return car->turnInnerPercent;
}
