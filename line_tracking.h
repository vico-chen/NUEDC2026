#ifndef LINE_TRACKING_H_
#define LINE_TRACKING_H_

#include "car_control.h"
#include "grayscale_sensor.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 巡线控制器的外部依赖。
 * 串口回调只用于可选的调试输出，不参与正常巡线控制。
 */
typedef struct {
    CarControl *car;
    void (*sendString)(const char *text);
    void (*sendInt32)(int32_t value);
} LineTracking_Config;

/* 巡线控制器的全部运行状态，不再散落在 main.c 的全局变量中。 */
typedef struct {
    LineTracking_Config config;
    bool enabled;
    bool debugEnabled;
    bool filterReady;
    int16_t baseSpeedRpm;
    float integral;
    int16_t lastError;
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

uint8_t LineTracking_readActiveMask(void);
uint8_t LineTracking_readActiveCount(void);

#endif /* LINE_TRACKING_H_ */
