#ifndef LINE_TRACKING_H_
#define LINE_TRACKING_H_

#include "car_control.h"
#include "grayscale_sensor.h"

#include <stdbool.h>
#include <stdint.h>

/* 巡线模块的外部依赖；串口回调只在调试输出时使用。 */
typedef struct {
    CarControl *car;
    void (*sendString)(const char *text);
    void (*sendInt32)(int32_t value);
} LineTracking_Config;

/*
 * 巡线控制器运行状态。
 * 把传感器消抖、误差滤波和 PID 历史量集中保存，避免散落在 main.c。
 */
typedef struct {
    LineTracking_Config config;
    bool enabled;
    bool debugEnabled;
    bool sensorFilterReady;
    bool errorFilterReady;
    int16_t baseSpeedRpm;
    float integral;
    float filteredError;
    int16_t lastError;
    int16_t appliedOffsetRpm;
    uint8_t activeMask;
    uint8_t activeCount;
    uint8_t debugDivider;
    uint8_t filteredValues[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t pendingValues[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t pendingCount[GRAYSCALE_SENSOR_CHANNELS];
} LineTracking;

void LineTracking_init(LineTracking *tracking,
    const LineTracking_Config *config, int16_t defaultSpeedRpm);
void LineTracking_reset(LineTracking *tracking);
void LineTracking_update(LineTracking *tracking);

void LineTracking_setEnabled(LineTracking *tracking, bool enabled);
bool LineTracking_isEnabled(const LineTracking *tracking);
void LineTracking_setDebugEnabled(LineTracking *tracking, bool enabled);
void LineTracking_setSpeed(LineTracking *tracking, int16_t speedRpm);
int16_t LineTracking_getSpeed(const LineTracking *tracking);

/* 返回最近一次滤波后的传感器状态，不会再次访问灰度模块。 */
uint8_t LineTracking_getActiveMask(const LineTracking *tracking);
uint8_t LineTracking_getActiveCount(const LineTracking *tracking);

#endif /* LINE_TRACKING_H_ */
