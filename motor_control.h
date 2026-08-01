#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <ti/driverlib/driverlib.h>

/* 单个电机的硬件映射、编码器规格以及闭环 PID 参数。 */
typedef struct {
    /* PWM 定时器与比较通道。 */
    GPTIMER_Regs *pwmInstance;
    DL_TIMER_CC_INDEX pwmChannel;
    /* H 桥方向引脚、编码器 AB 相引脚。 */
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
    /* 增量式 PID 参数及驱动输出限幅。 */
    float kp;
    float ki;
    float kd;
    float outputMaxPercent;
    float zeroSpeedBrakeMaxPercent;
} MotorControl_Config;

/* 串口状态快照；speedRpmTimes10 的单位是 0.1 RPM。 */
typedef struct {
    int16_t targetRpm;
    int32_t speedRpmTimes10;
    int32_t encoderCounts;
    int16_t pwmPercent;
} MotorControl_Status;

/* 运行时状态；volatile 字段在前台和中断之间共享。 */
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
    /* 前台命令处理程序写入，10 ms 定时中断读取。 */
    volatile bool coastMode;
} MotorControl;

/* 初始化、设置目标转速、滑行、处理编码器并周期更新 PID。 */
void MotorControl_init(
    MotorControl *motor, const MotorControl_Config *config);
void MotorControl_setTargetRpm(MotorControl *motor, int16_t targetRpm);
void MotorControl_coast(MotorControl *motor);
void MotorControl_handleEncoderEdge(MotorControl *motor);
void MotorControl_update(MotorControl *motor);
/* 取得一帧累计状态；没有新状态时返回 false。 */
bool MotorControl_takeStatus(
    MotorControl *motor, MotorControl_Status *status);
/* 非消费式读取最近一个状态周期的实际轮速，单位为 0.1 RPM。 */
int32_t MotorControl_getLatestSpeedRpmTimes10(
    const MotorControl *motor);

#endif
