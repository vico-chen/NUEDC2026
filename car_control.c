#include "car_control.h"

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

void CarControl_init(CarControl *car, const CarControl_Config *config)
{
    car->config = *config;
    car->speedRpm = config->defaultSpeedRpm;
    car->turnInnerPercent = config->defaultTurnInnerPercent;
    CarControl_stop(car);
}

void CarControl_stop(CarControl *car)
{
    CarControl_setWheelRpm(car, 0, 0, 0, 0);
}

void CarControl_emergencyStop(CarControl *car)
{
    /*
     * Do not use CarControl_stop() here. A zero speed target deliberately
     * enters the closed-loop reverse-braking branch, which is unsuitable for
     * an emergency command if an encoder is noisy or has the wrong polarity.
     */
    CarControl_coast(car);
}

void CarControl_coast(CarControl *car)
{
    MotorControl_coast(car->config.rightRearMotor);
    MotorControl_coast(car->config.rightFrontMotor);
    MotorControl_coast(car->config.leftFrontMotor);
    MotorControl_coast(car->config.leftRearMotor);
}

void CarControl_setMotion(CarControl *car, CarControl_Motion motion,
    int16_t speedRpm, uint8_t turnInnerPercent)
{
    int16_t innerWheelRpm;

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
    innerWheelRpm = (int16_t) (((int32_t) speedRpm *
        turnInnerPercent) / 100);

    switch (motion) {
        case CAR_CONTROL_FORWARD:
            CarControl_setWheelRpm(
                car, speedRpm, speedRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_BACKWARD:
            CarControl_setWheelRpm(
                car, -speedRpm, -speedRpm, -speedRpm, -speedRpm);
            break;
        case CAR_CONTROL_PIVOT_LEFT:
            CarControl_setWheelRpm(
                car, -speedRpm, -speedRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_PIVOT_RIGHT:
            CarControl_setWheelRpm(
                car, speedRpm, speedRpm, -speedRpm, -speedRpm);
            break;
        case CAR_CONTROL_FORWARD_LEFT:
            CarControl_setWheelRpm(
                car, innerWheelRpm, innerWheelRpm, speedRpm, speedRpm);
            break;
        case CAR_CONTROL_FORWARD_RIGHT:
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
