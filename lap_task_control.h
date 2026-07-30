#ifndef LAP_TASK_CONTROL_H_
#define LAP_TASK_CONTROL_H_

#include "line_tracking.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 环形巡线任务的可调参数。
 * 所有 Samples 参数均以 10 ms 状态机调度周期为单位。
 */
typedef struct {
    int16_t cruiseRpm;                    /* 正常巡线目标转速 */
    uint16_t accelerationSamples;         /* 从 0 加速至巡航速度的周期数 */
    uint8_t intersectionActiveThreshold;  /* 判定十字路口所需有效通道数 */
    uint8_t intersectionConfirmSamples;   /* 路口连续确认次数 */
    uint16_t finishAdvanceMm;             /* 扫到终点后继续前进的距离 */
    uint16_t wheelDiameterMm;             /* 编码器距离换算使用的有效轮径 */
    int16_t decelerationEndRpm;           /* 缓慢减速结束时的目标速度 */
    uint16_t decelerationSamples;         /* 从巡航速度降至结束速度的周期数 */
    uint16_t brakeMinimumSamples;         /* 制动后至少等待的周期数 */
    uint16_t brakeTimeoutSamples;         /* 停车确认的最大等待周期数 */
} LapTask_Profile;

/* 环形任务依赖的底层控制对象。 */
typedef struct {
    CarControl *car;                 /* 小车运动控制对象 */
    LineTracking *lineTracking;      /* 巡线控制对象 */
    MotorControl *motors[4];         /* 用于读取四轮编码器速度 */
    void (*log)(const char *text);   /* 状态变化日志回调 */
} LapTask_Config;

/* 环形任务完整状态流程。 */
typedef enum {
    LAP_TASK_IDLE,            /* 空闲，等待启动 */
    LAP_TASK_FOLLOW,          /* 加速巡线并寻找终点十字路口 */
    LAP_TASK_FINISH_ADVANCE,  /* 扫到终点后继续前进指定距离 */
    LAP_TASK_DECELERATE,      /* 保持巡线并逐步降低速度 */
    LAP_TASK_BRAKE,           /* 目标速度归零并等待四轮停稳 */
    LAP_TASK_DONE,            /* 正常完成 */
    LAP_TASK_FAULT            /* 停车超时等故障 */
} LapTask_State;

/* 环形任务运行时计数器和编码器累计值。 */
typedef struct {
    LapTask_Config config;                    /* 底层依赖 */
    const LapTask_Profile *profile;           /* 当前任务使用的参数表 */
    LapTask_State state;                      /* 当前状态机阶段 */
    uint8_t taskNumber;                       /* 当前任务编号 */
    uint8_t intersectionConfirmCount;         /* 十字路口确认计数 */
    uint16_t accelerationSamples;             /* 已完成的加速周期 */
    uint16_t decelerationSamples;             /* 已完成的减速周期 */
    uint16_t brakeSamples;                    /* 已等待的制动周期 */
    uint32_t finishEncoderCount;              /* 终点后累计的四轮计数 */
    uint32_t finishTargetEncoderCount;        /* 继续前进距离对应的目标计数 */
} LapTaskControl;

/* 环形任务初始化、启动、复位、周期更新及状态查询接口。 */
void LapTaskControl_init(LapTaskControl *control,
    const LapTask_Config *config);
bool LapTaskControl_start(LapTaskControl *control, uint8_t taskNumber,
    const LapTask_Profile *profile);
void LapTaskControl_reset(LapTaskControl *control);
void LapTaskControl_update(LapTaskControl *control);
bool LapTaskControl_isRunning(const LapTaskControl *control);
bool LapTaskControl_isFinished(const LapTaskControl *control);

#endif /* LAP_TASK_CONTROL_H_ */
