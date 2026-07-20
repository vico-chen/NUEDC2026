#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <ti/driverlib/driverlib.h>

typedef struct {
    GPTIMER_Regs *pwmInstance;
    DL_TIMER_CC_INDEX pwmChannel;
    GPIO_Regs *directionIn1Port;
    GPIO_Regs *directionIn2Port;
    uint32_t directionIn1Pin;
    uint32_t directionIn2Pin;
    GPIO_Regs *encoderPhaseAPort;
    GPIO_Regs *encoderPhaseBPort;
    uint32_t encoderPhaseAPin;
    uint32_t encoderPhaseBPin;
    uint16_t pwmPeriodCounts;
    uint16_t encoderPpr;
    uint16_t gearRatio;
    uint8_t encoderDecodeMultiplier;
    uint16_t sampleRateHz;
    uint8_t reportSamples;
    int16_t maxTargetRpm;
    int32_t zeroSpeedDeadbandCounts;
    float kp;
    float ki;
    float kd;
    float outputMaxPercent;
    float zeroSpeedBrakeMaxPercent;
} MotorControl_Config;

typedef struct {
    int16_t targetRpm;
    int32_t speedRpmTimes10;
    int32_t encoderCounts;
    int16_t pwmPercent;
} MotorControl_Status;

typedef struct {
    MotorControl_Config config;
    volatile int16_t targetRpm;
    volatile int16_t pwmPercent;
    volatile int32_t encoderCount;
    volatile int32_t speedCountsPerSample;
    volatile int32_t reportCountAccumulator;
    volatile int32_t reportCounts;
    volatile uint8_t reportDivider;
    volatile bool statusReady;
    float pidOutput;
    float pidLastError;
    float pidPreviousError;
    int8_t pidDirection;
    int8_t zeroBrakeDirection;
    bool coastMode;
} MotorControl;

void MotorControl_init(
    MotorControl *motor, const MotorControl_Config *config);
void MotorControl_setTargetRpm(MotorControl *motor, int16_t targetRpm);
void MotorControl_coast(MotorControl *motor);
void MotorControl_handleEncoderEdge(MotorControl *motor);
void MotorControl_update(MotorControl *motor);
bool MotorControl_takeStatus(
    MotorControl *motor, MotorControl_Status *status);

#endif
