#include "line_tracking.h"

#define LINE_ACTIVE_LEVEL                    (1U)
#define LINE_PID_KP                          (2.5f)
#define LINE_PID_KI                          (0.01f)
#define LINE_PID_KD                          (0.0f)
#define LINE_PID_INTEGRAL_LIMIT              (2000.0f)
#define LINE_ERROR_FILTER_ALPHA              (0.55f)
#define LINE_ERROR_DEADBAND                  (5)
#define LINE_LOST_ERROR                      (60)
#define LINE_OFFSET_DEADBAND_RPM             (8)
#define LINE_MAX_OFFSET_PERCENT              (85U)
#define LINE_MIN_INNER_PERCENT               (15U)
#define LINE_MAX_OFFSET_STEP_RPM             (18)
#define LINE_CORNER_ERROR_THRESHOLD           (35)
#define LINE_CORNER_MIN_OFFSET_PERCENT        (70U)
#define LINE_SHARP_ERROR_THRESHOLD            (55)
#define LINE_SHARP_MIN_OFFSET_PERCENT         (85U)
#define LINE_SENSOR_DEBOUNCE_SAMPLES         (2U)
#define LINE_DEBUG_PERIOD_SAMPLES            (10U)

static int16_t LineTracking_roundFloat(float value)
{
    return (value >= 0.0f) ? (int16_t) (value + 0.5f)
                           : (int16_t) (value - 0.5f);
}

static int16_t LineTracking_slew(int16_t current, int16_t target)
{
    int16_t delta = (int16_t) (target - current);

    if (delta > LINE_MAX_OFFSET_STEP_RPM) {
        return (int16_t) (current + LINE_MAX_OFFSET_STEP_RPM);
    }
    if (delta < -LINE_MAX_OFFSET_STEP_RPM) {
        return (int16_t) (current - LINE_MAX_OFFSET_STEP_RPM);
    }
    return target;
}

void LineTracking_reset(LineTracking *tracking)
{
    uint8_t i;

    tracking->integral = 0.0f;
    tracking->filteredError = 0.0f;
    tracking->lastError = 0;
    tracking->appliedOffsetRpm = 0;
    tracking->lastLineDirection = 0;
    tracking->activeMask = 0U;
    tracking->activeCount = 0U;
    tracking->debugDivider = 0U;
    tracking->sensorFilterReady = false;
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
    *tracking = (LineTracking) {0};
    tracking->config = *config;
    tracking->baseSpeedRpm = defaultSpeedRpm;
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

uint8_t LineTracking_getActiveMask(const LineTracking *tracking)
{
    return tracking->activeMask;
}

uint8_t LineTracking_getActiveCount(const LineTracking *tracking)
{
    return tracking->activeCount;
}

static void LineTracking_filterSensors(LineTracking *tracking,
    const uint8_t raw[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t i;

    if (!tracking->sensorFilterReady) {
        for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
            tracking->filteredValues[i] = raw[i];
            tracking->pendingValues[i] = raw[i];
            tracking->pendingCount[i] = 0U;
        }
        tracking->sensorFilterReady = true;
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
    }
}

static int16_t LineTracking_computeError(LineTracking *tracking)
{
    static const int16_t weights[GRAYSCALE_SENSOR_CHANNELS] = {
        -60, -38, -18, 0, 0, 18, 38, 60
    };
    int32_t weightedSum = 0;
    uint8_t centerCount = 0U;
    uint8_t outerCount = 0U;
    uint8_t i;
    int16_t error;

    tracking->activeMask = 0U;
    tracking->activeCount = 0U;
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (tracking->filteredValues[i] == LINE_ACTIVE_LEVEL) {
            tracking->activeMask |= (uint8_t) (1U << i);
            tracking->activeCount++;
            weightedSum += weights[i];
            if ((i == 3U) || (i == 4U)) {
                centerCount++;
            } else {
                outerCount++;
            }
        }
    }

    if ((centerCount > 0U) && (outerCount == 0U)) {
        return 0;
    }
    if (tracking->activeCount > 0U) {
        error = (int16_t) (weightedSum / tracking->activeCount);
        if ((error >= -LINE_ERROR_DEADBAND) &&
            (error <= LINE_ERROR_DEADBAND)) {
            return 0;
        }
        tracking->lastLineDirection = (error > 0) ? 1 : -1;
        return error;
    }

    if (tracking->lastLineDirection > 0) {
        return LINE_LOST_ERROR;
    }
    if (tracking->lastLineDirection < 0) {
        return -LINE_LOST_ERROR;
    }
    return 0;
}

static void LineTracking_applyMotion(LineTracking *tracking,
    int16_t offsetRpm)
{
    int16_t speedRpm = tracking->baseSpeedRpm;
    int16_t offsetAbs = (offsetRpm < 0) ? -offsetRpm : offsetRpm;
    int16_t innerRpm;
    uint8_t innerPercent;

    if (speedRpm <= 0) {
        CarControl_stop(tracking->config.car);
        return;
    }

    if (offsetAbs <= LINE_OFFSET_DEADBAND_RPM) {
        CarControl_setMotion(tracking->config.car, CAR_CONTROL_FORWARD,
            speedRpm, 100U);
        return;
    }

    innerRpm = (int16_t) (speedRpm - offsetAbs);
    innerPercent = (uint8_t) (((int32_t) innerRpm * 100) / speedRpm);
    if (innerPercent < LINE_MIN_INNER_PERCENT) {
        innerPercent = LINE_MIN_INNER_PERCENT;
    }
    if (innerPercent > 100U) {
        innerPercent = 100U;
    }

    CarControl_setMotion(tracking->config.car,
        (offsetRpm < 0) ? CAR_CONTROL_FORWARD_LEFT
                        : CAR_CONTROL_FORWARD_RIGHT,
        speedRpm, innerPercent);
}

void LineTracking_update(LineTracking *tracking)
{
    uint8_t raw[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t i;
    int16_t rawError;
    int16_t error;
    int16_t desiredOffset;
    int16_t maxOffset;
    int16_t minimumCornerOffset;
    int16_t rawErrorAbs;
    float derivative;
    float pid;

    if (!tracking->enabled) {
        return;
    }

    Grayscale_Sensor_ReadAll(raw);
    LineTracking_filterSensors(tracking, raw);
    rawError = LineTracking_computeError(tracking);

    if (!tracking->errorFilterReady) {
        tracking->filteredError = rawError;
        tracking->errorFilterReady = true;
    } else {
        tracking->filteredError += LINE_ERROR_FILTER_ALPHA *
            ((float) rawError - tracking->filteredError);
    }
    error = LineTracking_roundFloat(tracking->filteredError);

    if ((tracking->activeCount == 0U) || (rawError == 0)) {
        tracking->integral = 0.0f;
    } else {
        tracking->integral += error;
        if (tracking->integral > LINE_PID_INTEGRAL_LIMIT) {
            tracking->integral = LINE_PID_INTEGRAL_LIMIT;
        } else if (tracking->integral < -LINE_PID_INTEGRAL_LIMIT) {
            tracking->integral = -LINE_PID_INTEGRAL_LIMIT;
        }
    }

    derivative = (float) (error - tracking->lastError);
    pid = LINE_PID_KP * error +
          LINE_PID_KI * tracking->integral +
          LINE_PID_KD * derivative;

    maxOffset = (int16_t) (((int32_t) tracking->baseSpeedRpm *
        LINE_MAX_OFFSET_PERCENT) / 100);
    if (maxOffset < 0) {
        maxOffset = 0;
    }
    if (pid > maxOffset) {
        pid = maxOffset;
    } else if (pid < -maxOffset) {
        pid = -maxOffset;
    }

    desiredOffset = LineTracking_roundFloat(pid);

    /*
     * 普通 PID 在最外侧探头刚压线时修正量仍可能偏小，车辆会以过大的
     * 转弯半径冲出弯道。大误差区加入按车速缩放的最小转向量：中等弯道
     * 至少使用 70% 差速，最外侧探头或完全丢线时使用 85%。修正量仍经过
     * 18 RPM/周期的斜率限制，内轮也始终保持正转，不会突然反刹卡顿。
     */
    rawErrorAbs = (rawError < 0) ? -rawError : rawError;
    minimumCornerOffset = 0;
    if ((tracking->activeCount == 0U) ||
        (rawErrorAbs >= LINE_SHARP_ERROR_THRESHOLD)) {
        minimumCornerOffset = (int16_t) (((int32_t)
            tracking->baseSpeedRpm * LINE_SHARP_MIN_OFFSET_PERCENT) /
            100);
    } else if (rawErrorAbs >= LINE_CORNER_ERROR_THRESHOLD) {
        minimumCornerOffset = (int16_t) (((int32_t)
            tracking->baseSpeedRpm * LINE_CORNER_MIN_OFFSET_PERCENT) /
            100);
    }
    if (minimumCornerOffset > 0) {
        if ((rawError < 0) &&
            (desiredOffset > -minimumCornerOffset)) {
            desiredOffset = -minimumCornerOffset;
        } else if ((rawError > 0) &&
                   (desiredOffset < minimumCornerOffset)) {
            desiredOffset = minimumCornerOffset;
        }
    }

    tracking->appliedOffsetRpm = LineTracking_slew(
        tracking->appliedOffsetRpm, desiredOffset);
    tracking->lastError = error;
    LineTracking_applyMotion(tracking, tracking->appliedOffsetRpm);

    if (tracking->debugEnabled &&
        (tracking->config.sendString != 0) &&
        (tracking->config.sendInt32 != 0)) {
        tracking->debugDivider++;
        if (tracking->debugDivider >= LINE_DEBUG_PERIOD_SAMPLES) {
            tracking->debugDivider = 0U;
            tracking->config.sendString("LINE err=");
            tracking->config.sendInt32(error);
            tracking->config.sendString(" offset=");
            tracking->config.sendInt32(tracking->appliedOffsetRpm);
            tracking->config.sendString(" inner=");
            tracking->config.sendInt32(
                CarControl_getTurnInnerPercent(tracking->config.car));
            tracking->config.sendString(" sensors=");
            for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
                tracking->config.sendInt32(tracking->filteredValues[i]);
            }
            tracking->config.sendString("\r\n");
        }
    }
}
