#ifndef STRAIGHT_TASK_CONTROL_H_
#define STRAIGHT_TASK_CONTROL_H_

#include "car_control.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 编码器定距直行参数。
 * 距离和轮径单位均为 mm，Samples 参数单位为 10 ms。
 */
typedef struct {
    uint16_t targetDistanceMm;        /* 总目标距离，Task4 当前为 1500 mm */
    uint16_t wheelDiameterMm;         /* 编码器距离换算使用的有效轮径 */
    int16_t cruiseRpm;                /* 中段匀速行驶速度 */
    uint16_t accelerationSamples;     /* 起步加速周期数 */
    uint16_t decelerationDistanceMm;  /* 距终点多远开始减速 */
    int16_t decelerationEndRpm;       /* 到达终点前的最低目标速度 */
    uint16_t brakeMinimumSamples;     /* 制动后至少等待的周期数 */
    uint16_t brakeTimeoutSamples;     /* 停车确认超时周期数 */
} StraightTask_Profile;

/* 直行模块依赖的车辆和四轮电机对象。 */
typedef struct {
    CarControl *car;                 /* 小车运动控制对象 */
    MotorControl *motors[4];         /* 用于累计行驶距离 */
    void (*log)(const char *text);   /* 状态变化日志回调 */
} StraightTask_Config;

/* 定距直行任务状态。 */
typedef enum {
    STRAIGHT_TASK_IDLE,       /* 空闲，等待启动 */
    STRAIGHT_TASK_RUNNING,    /* 加速、匀速或按剩余距离减速 */
    STRAIGHT_TASK_BRAKE,      /* 到达目标距离后制动 */
    STRAIGHT_TASK_DONE,       /* 正常完成 */
    STRAIGHT_TASK_FAULT       /* 停车超时 */
} StraightTask_State;

/* 直行任务运行时数据。 */
typedef struct {
    StraightTask_Config config;              /* 底层依赖 */
    const StraightTask_Profile *profile;     /* 当前使用的参数表 */
    StraightTask_State state;                /* 当前状态 */
    uint16_t accelerationSamples;            /* 已完成的加速周期 */
    uint16_t brakeSamples;                   /* 已等待的制动周期 */
    uint32_t encoderCount;                   /* 当前四轮累计编码器计数 */
    uint32_t targetEncoderCount;             /* 1.5 m 对应的目标计数 */
    uint32_t decelerationEncoderCount;       /* 减速距离对应的计数 */
} StraightTaskControl;

/* 直行任务初始化、启动、复位、周期更新及状态查询接口。 */
void StraightTaskControl_init(StraightTaskControl *control,
    const StraightTask_Config *config);
bool StraightTaskControl_start(StraightTaskControl *control,
    const StraightTask_Profile *profile);
void StraightTaskControl_reset(StraightTaskControl *control);
void StraightTaskControl_update(StraightTaskControl *control);
bool StraightTaskControl_isRunning(const StraightTaskControl *control);
bool StraightTaskControl_isFinished(const StraightTaskControl *control);

#endif /* STRAIGHT_TASK_CONTROL_H_ */
