#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <ti/driverlib/driverlib.h>

/*
 * 单电机闭环控制模块
 *
 * 每个 MotorControl 对象对应一台直流减速电机，负责接收目标转速、
 * 软件正交解码、10 ms 速度 PID、TB6612 方向/PWM 输出及状态统计。
 */

/* 与具体电机和硬件连接相关的常量配置。 */
typedef struct {
    /* PWM 定时器实例、比较通道及计数周期。 */
    GPTIMER_Regs *pwmInstance;
    DL_TIMER_CC_INDEX pwmChannel;
    /* TB6612 的 IN1/IN2 方向控制端口和引脚。 */
    GPIO_Regs *directionIn1Port;
    GPIO_Regs *directionIn2Port;
    uint32_t directionIn1Pin;
    uint32_t directionIn2Pin;
    /* 编码器 A/B 两相；B 相配置为双边沿中断。 */
    GPIO_Regs *encoderPhaseAPort;
    GPIO_Regs *encoderPhaseBPort;
    uint32_t encoderPhaseAPin;
    uint32_t encoderPhaseBPin;
    uint16_t pwmPeriodCounts;
    /* 输出轴每圈计数 = PPR × 减速比 × 解码倍频。 */
    uint16_t encoderPpr;
    uint16_t gearRatio;
    uint8_t encoderDecodeMultiplier;
    /* PID 更新频率和状态汇报所累计的 PID 周期数。 */
    uint16_t sampleRateHz;
    uint8_t reportSamples;
    int16_t maxTargetRpm;
    /* 目标为 0 时，小于该计数阈值即认为已经停止。 */
    int32_t zeroSpeedDeadbandCounts;
    /*
     * 10 ms 编码器增量的一阶 IIR 滤波系数。
     * 0 < alpha <= 1；数值越小，低速亚脉冲估计越平滑。
     */
    float speedFilterAlpha;
    /* 增量式 PID 参数。 */
    float kp;
    float ki;
    float kd;
    /* 正常驱动最大占空比。 */
    float outputMaxPercent;
    /* 行驶中仅允许轻微反向制动，停车时才使用较强制动力。 */
    float runningBrakeMaxPercent;
    float stopBrakeMaxPercent;
} MotorControl_Config;

/* 串口状态输出使用的只读快照。 */
typedef struct {
    int16_t targetRpm;
    int32_t speedRpmTimes10;
    int32_t encoderCounts;
    int16_t pwmPercent;
} MotorControl_Status;

/* 单电机运行时状态；中断和前台会共同访问部分 volatile 字段。 */
typedef struct {
    MotorControl_Config config;
    /* 前台写目标，10 ms 定时中断读取并执行控制。 */
    volatile int16_t targetRpm;
    volatile int16_t pwmPercent;
    /* 编码器 ISR 累加，PID 周期读取后清零。 */
    volatile int32_t encoderCount;
    volatile int32_t speedCountsPerSample;
    /* 100 ms 状态汇报累计器。 */
    volatile int32_t reportCountAccumulator;
    volatile int32_t reportCounts;
    volatile uint8_t reportDivider;
    volatile bool statusReady;
    /* PID 使用的低通测速结果。 */
    float filteredSpeedCountsPerSample;
    bool speedFilterReady;
    /* 增量式 PID 的历史状态。 */
    float pidOutput;
    float pidLastError;
    float pidPreviousError;
    int8_t pidDirection;
    /* true 表示立即撤销驱动，不执行目标为 0 时的主动制动。 */
    volatile bool coastMode;
} MotorControl;

/* 初始化对象，并把 PWM 和方向输出置为停止状态。 */
void MotorControl_init(
    MotorControl *motor, const MotorControl_Config *config);
/* 设置目标转速；正负号表示电机自身方向。 */
void MotorControl_setTargetRpm(MotorControl *motor, int16_t targetRpm);
/* 立即关闭驱动并清空 PID。 */
void MotorControl_coast(MotorControl *motor);
/* 编码器 B 相双边沿中断入口。 */
void MotorControl_handleEncoderEdge(MotorControl *motor);
/* 每 10 ms 调用：测速、滤波、PID 和 PWM 输出。 */
void MotorControl_update(MotorControl *motor);
/* 取得最新 100 ms 状态；无新数据时返回 false。 */
bool MotorControl_takeStatus(
    MotorControl *motor, MotorControl_Status *status);

#endif
