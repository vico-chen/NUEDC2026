#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "motor_control.h"

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

typedef struct {
    CarControl_Config config;
    int16_t speedRpm;
    uint8_t turnInnerPercent;
} CarControl;

void CarControl_init(CarControl *car, const CarControl_Config *config);
void CarControl_stop(CarControl *car);
void CarControl_coast(CarControl *car);
void CarControl_setMotion(CarControl *car, CarControl_Motion motion,
    int16_t speedRpm, uint8_t turnInnerPercent);
int16_t CarControl_getSpeedRpm(const CarControl *car);
uint8_t CarControl_getTurnInnerPercent(const CarControl *car);

#endif
