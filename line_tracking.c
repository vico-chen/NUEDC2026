#include "line_tracking.h"

/* 八路灰度模块检测到黑线时的有效电平。 */
#define LINE_TRACKING_ACTIVE_LEVEL (1U)

/* 巡线 PID 参数。 */
#define LINE_PID_KP (3.0f)
#define LINE_PID_KI (0.01f)
#define LINE_PID_KD (0.0f)
#define LINE_PID_INTEGRAL_LIMIT (2000.0f)
#define LINE_PID_DEADBAND_RPM_OFFSET (8)
#define LINE_ERR_DEADBAND (5)
#define LINE_ERR_ABS_FALLBACK (30)
#define LINE_DEBUG_PRINT_PERIOD_SAMPLES (10U)
#define LINE_SENSOR_DEBOUNCE_SAMPLES (2U)
/* 一阶低通滤波：每次采用 30% 新误差，保留 70% 历史误差。 */
#define LINE_ERROR_FILTER_ALPHA (1.00f)

/* 清除 PID、传感器滤波状态和丢线方向记忆。 */
void LineTracking_reset(LineTracking *tracking)
{
    uint8_t i;

    tracking->integral = 0.0f;
    tracking->filteredError = 0.0f;
    tracking->lastError = 0;
    tracking->lastLineDirection = 0;
    tracking->debugDivider = 0U;
    tracking->filterReady = false;
    tracking->errorFilterReady = false;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        tracking->filteredValues[i] = 0U;
        tracking->pendingValues[i] = 0U;
        tracking->pendingCount[i] = 0U;
    }
}

void LineTracking_init(LineTracking *tracking,
    const LineTracking_Config *config, int16_t defaultSpeedRpm)
{
    tracking->config = *config;
    tracking->enabled = false;
    tracking->debugEnabled = false;
    tracking->baseSpeedRpm = defaultSpeedRpm;
    tracking->lostLineRecoveryRpmOffset = 0;
    LineTracking_reset(tracking);
}

void LineTracking_setEnabled(LineTracking *tracking, bool enabled)
{
    tracking->enabled = enabled;
}

bool LineTracking_isEnabled(const LineTracking *tracking)
{
    return tracking->enabled;
}

void LineTracking_setDebugEnabled(LineTracking *tracking, bool enabled)
{
    tracking->debugEnabled = enabled;
}

void LineTracking_setSpeed(LineTracking *tracking, int16_t speedRpm)
{
    tracking->baseSpeedRpm = speedRpm;
}

int16_t LineTracking_getSpeed(const LineTracking *tracking)
{
    return tracking->baseSpeedRpm;
}

void LineTracking_setLostLineRecoveryRpmOffset(
    LineTracking *tracking, int16_t rpmOffset)
{
    /* 负数没有物理意义；设置为 0 可关闭终极回正增强。 */
    tracking->lostLineRecoveryRpmOffset =
        (rpmOffset > 0) ? rpmOffset : 0;
}

/* 对八路数字灰度结果进行软件消抖。 */
static void LineTracking_filterSensors(LineTracking *tracking,
    const uint8_t raw[GRAYSCALE_SENSOR_CHANNELS],
    uint8_t filtered[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t i;

    if (!tracking->filterReady) {
        for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
            tracking->filteredValues[i] = raw[i];
            tracking->pendingValues[i] = raw[i];
            tracking->pendingCount[i] = 0U;
            filtered[i] = raw[i];
        }
        tracking->filterReady = true;
        return;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (raw[i] == tracking->filteredValues[i]) {
            tracking->pendingValues[i] = raw[i];
            tracking->pendingCount[i] = 0U;
        } else if (raw[i] == tracking->pendingValues[i]) {
            if (tracking->pendingCount[i] < 255U) {
                tracking->pendingCount[i]++;
            }
            if (tracking->pendingCount[i] >=
                LINE_SENSOR_DEBOUNCE_SAMPLES) {
                tracking->filteredValues[i] = raw[i];
                tracking->pendingCount[i] = 0U;
            }
        } else {
            tracking->pendingValues[i] = raw[i];
            tracking->pendingCount[i] = 1U;
        }
        filtered[i] = tracking->filteredValues[i];
    }
}

/*
 * 根据 X1～X8 的横向位置计算偏差。
 * X1 位于最左侧，X8 位于最右侧；负数向左修正，正数向右修正。
 */
static int16_t LineTracking_computeError(LineTracking *tracking,
    const uint8_t values[GRAYSCALE_SENSOR_CHANNELS])
{
    static const int16_t weights[GRAYSCALE_SENSOR_CHANNELS] = {
        -60, -40, -15, 0, 0, 15, 40, 60
    };
    int32_t weightedSum = 0;
    uint8_t activeCount = 0U;
    uint8_t outerActive = 0U;
    uint8_t centerActive = 0U;
    uint8_t i;
    int16_t error;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (values[i] == LINE_TRACKING_ACTIVE_LEVEL) {
            weightedSum += (int32_t) weights[i];
            activeCount++;
            if ((i == 3U) || (i == 4U)) {
                centerActive++;
            } else {
                outerActive++;
            }
        }
    }

    /*
     * 恢复红外改造前的八路逻辑：只有中央 X4/X5 压线时，
     * 直接认为车辆已经居中，避免中央两路边缘抖动造成左右摆动。
     */
    if ((centerActive > 0U) && (outerActive == 0U)) {
        return 0;
    }

    if (activeCount > 0U) {
        error = (int16_t) (weightedSum / (int32_t) activeCount);
        if ((error <= LINE_ERR_DEADBAND) &&
            (error >= -LINE_ERR_DEADBAND)) {
            return 0;
        }
        tracking->lastLineDirection = (error > 0) ? 1 : -1;
        return error;
    }

    /* 丢线后沿最后一次看到线路时的偏差方向继续寻找。 */
    if (tracking->lastLineDirection > 0) {
        return LINE_ERR_ABS_FALLBACK;
    }
    if (tracking->lastLineDirection < 0) {
        return (int16_t) (-LINE_ERR_ABS_FALLBACK);
    }
    return 0;
}

/* 把 PID 输出转换成左右轮差速。 */
static void LineTracking_applyMotion(LineTracking *tracking,
    int16_t pidRpmOffset)
{
    int16_t speedRpm = tracking->baseSpeedRpm;
    int16_t pidAbs = (pidRpmOffset < 0) ?
        (int16_t) (-pidRpmOffset) : pidRpmOffset;
    int16_t innerRpm;
    uint8_t innerPercent;
    CarControl_Motion motion;

    if (speedRpm <= 0) {
        CarControl_stop(tracking->config.car);
        return;
    }

    if (pidAbs <= LINE_PID_DEADBAND_RPM_OFFSET) {
        CarControl_setMotion(tracking->config.car, CAR_CONTROL_FORWARD,
            speedRpm, CarControl_getTurnInnerPercent(tracking->config.car));
        return;
    }

    if (pidAbs > speedRpm) {
        pidAbs = speedRpm;
    }
    innerRpm = (int16_t) (speedRpm - pidAbs);
    if (innerRpm < 0) {
        innerRpm = 0;
    }

    innerPercent = (uint8_t) ((int32_t) innerRpm * 100 /
        (int32_t) speedRpm);
    if (innerPercent > 100U) {
        innerPercent = 100U;
    }
    if (innerPercent == 0U) {
        innerPercent = 1U;
    }

    motion = (pidRpmOffset < 0) ? CAR_CONTROL_FORWARD_LEFT
                                 : CAR_CONTROL_FORWARD_RIGHT;
    CarControl_setMotion(tracking->config.car, motion,
        speedRpm, innerPercent);
}

void LineTracking_update(LineTracking *tracking)
{
    uint8_t rawValues[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t values[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t activeCount = 0U;
    uint8_t i;
    int16_t rawError;
    int16_t error;
    float derivative;
    float pid;
    int16_t pidOffsetRpm;

    if (!tracking->enabled) {
        return;
    }

    Grayscale_Sensor_ReadAll(rawValues);
    LineTracking_filterSensors(tracking, rawValues, values);

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (values[i] == LINE_TRACKING_ACTIVE_LEVEL) {
            activeCount++;
        }
    }

    rawError = LineTracking_computeError(tracking, values);

    /*
     * 对离散灰度通道产生的阶跃误差做一阶低通滤波。
     * 首次采样直接采用当前误差，保证车辆启动时仍能及时响应弯道。
     */
    if (!tracking->errorFilterReady) {
        tracking->filteredError = (float) rawError;
        tracking->errorFilterReady = true;
    } else {
        tracking->filteredError += LINE_ERROR_FILTER_ALPHA *
            ((float) rawError - tracking->filteredError);
    }

    /* 将滤波后的浮点误差对称地四舍五入，供现有整数 PID 使用。 */
    if (tracking->filteredError >= 0.0f) {
        error = (int16_t) (tracking->filteredError + 0.5f);
    } else {
        error = (int16_t) (tracking->filteredError - 0.5f);
    }

    if ((activeCount == 0U) || (rawError == 0)) {
        tracking->integral = 0.0f;
    } else {
        tracking->integral += (float) error;
        if (tracking->integral > LINE_PID_INTEGRAL_LIMIT) {
            tracking->integral = LINE_PID_INTEGRAL_LIMIT;
        } else if (tracking->integral < -LINE_PID_INTEGRAL_LIMIT) {
            tracking->integral = -LINE_PID_INTEGRAL_LIMIT;
        }
    }

    derivative = (float) (error - tracking->lastError);
    pid = (LINE_PID_KP * (float) error) +
        (LINE_PID_KI * tracking->integral) +
        (LINE_PID_KD * derivative);

    /*
     * 八路全部无有效信息表示车辆已经跑出赛道。
     * 此时沿最后一次偏差方向继续搜索，并保证左右轮差速不低于
     * lostLineRecoveryRpmOffset，以获得更大的回正角速度。
     */
    if ((activeCount == 0U) && (rawError != 0) &&
        (tracking->lostLineRecoveryRpmOffset > 0)) {
        float recoveryOffset =
            (float) tracking->lostLineRecoveryRpmOffset;

        if ((rawError > 0) && (pid < recoveryOffset)) {
            pid = recoveryOffset;
        } else if ((rawError < 0) && (pid > -recoveryOffset)) {
            pid = -recoveryOffset;
        }
    }

    if (pid > (float) tracking->baseSpeedRpm) {
        pid = (float) tracking->baseSpeedRpm;
    } else if (pid < -(float) tracking->baseSpeedRpm) {
        pid = -(float) tracking->baseSpeedRpm;
    }

    pidOffsetRpm = (int16_t) pid;
    tracking->lastError = error;
    LineTracking_applyMotion(tracking, pidOffsetRpm);

    if (tracking->debugEnabled &&
        (tracking->config.sendString != 0) &&
        (tracking->config.sendInt32 != 0)) {
        tracking->debugDivider++;
        if (tracking->debugDivider >=
            LINE_DEBUG_PRINT_PERIOD_SAMPLES) {
            tracking->debugDivider = 0U;
            tracking->config.sendString("line:");
            tracking->config.sendInt32((int32_t) error);
            tracking->config.sendString(",");
            tracking->config.sendInt32((int32_t) pidOffsetRpm);
            for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
                tracking->config.sendString(",");
                tracking->config.sendInt32((int32_t) values[i]);
            }
            tracking->config.sendString("\r\n");
        }
    }
}

uint8_t LineTracking_readActiveMask(void)
{
    uint8_t values[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t activeMask = 0U;
    uint8_t i;

    Grayscale_Sensor_ReadAll(values);
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (values[i] == LINE_TRACKING_ACTIVE_LEVEL) {
            activeMask |= (uint8_t) (1U << i);
        }
    }
    return activeMask;
}

uint8_t LineTracking_readActiveCount(void)
{
    uint8_t mask = LineTracking_readActiveMask();
    uint8_t count = 0U;

    while (mask != 0U) {
        count = (uint8_t) (count + (mask & 0x01U));
        mask >>= 1U;
    }
    return count;
}
