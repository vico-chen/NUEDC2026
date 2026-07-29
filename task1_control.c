#include "task1_control.h"

/* 以下时间参数均按 10 ms 状态机周期计数。 */
#define TASK1_CRUISE_TARGET_RPM (150)
#define TASK1_ACCELERATION_SAMPLES (20U)
#define TASK1_INTERSECTION_THRESHOLD (6U)
#define TASK1_INTERSECTION_CONFIRM_SAMPLES (2U)
#define TASK1_ADVANCE_SAMPLES (16U)
#define TASK1_BRAKE_MIN_SAMPLES (15U)
#define TASK1_BRAKE_TIMEOUT_SAMPLES (80U)
#define TASK1_TURN_DEGREES (85.0f)
#define TASK1_TURN_RPM (100)
#define TASK1_RETURN_DEGREES (180.0f)
#define TASK1_BLANK_CONFIRM_SAMPLES (10U)
#define TASK1_REVERSE_RPM (75)
#define TASK1_REVERSE_SAMPLES (50U)
#define TASK1_FINAL_SETTLE_SAMPLES (5U)

static void Task1Control_log(Task1Control *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

/* 四个车轮都落入各自零速死区，才认为车辆已经停稳。 */
static bool Task1Control_carStopped(const Task1Control *control)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        int32_t counts = control->config.motors[i]->speedCountsPerSample;
        int32_t deadband =
            control->config.motors[i]->config.zeroSpeedDeadbandCounts;
        if ((counts > deadband) || (counts < -deadband)) {
            return false;
        }
    }
    return true;
}

/* 以固定步数将巡线速度从 0 平滑提升到目标 RPM。 */
static void Task1Control_applyRamp(Task1Control *control)
{
    if (control->accelerationSamples < TASK1_ACCELERATION_SAMPLES) {
        control->accelerationSamples++;
    }
    control->config.setLineTrackingSpeed((int16_t) (((int32_t)
        TASK1_CRUISE_TARGET_RPM * control->accelerationSamples) /
        TASK1_ACCELERATION_SAMPLES));
}

/* 八路全有效立即确认；六路以上需连续多帧确认十字路口。 */
static bool Task1Control_detectIntersection(Task1Control *control)
{
    uint8_t activeCount = control->config.readActiveChannelCount();

    if (activeCount == 8U) {
        control->intersectionPartialCount = 0U;
        return true;
    }
    if (activeCount >= TASK1_INTERSECTION_THRESHOLD) {
        if (control->intersectionPartialCount < 255U) {
            control->intersectionPartialCount++;
        }
        return (control->intersectionPartialCount >=
            TASK1_INTERSECTION_CONFIRM_SAMPLES);
    }
    control->intersectionPartialCount = 0U;
    return false;
}

static void Task1Control_start(Task1Control *control,
    TaskManager_Task1Endpoint endpoint)
{
    /* 锁定本次端点，清空各阶段计数并启动巡线。 */
    control->endpoint2 = (endpoint == TASK_MANAGER_TASK1_ENDPOINT_2);
    control->returnPhase = false;
    control->nextTurnRight = false;
    control->intersectionPartialCount = 0U;
    control->advanceSamples = 0U;
    control->accelerationSamples = 0U;
    control->blankCount = 0U;
    control->brakeSamples = 0U;
    control->reverseSamples = 0U;
    control->finalBrakeSettledCount = 0U;
    control->lineSeenAfterTurn = false;
    control->config.setLineTrackingSpeed(0);
    control->config.setLineTrackingEnabled(true);
    control->config.resetLineTracking();
    AngleTurnControl_cancel(control->config.angleTurn);
    control->state = TASK1_CONTROL_FOLLOW_INTERSECTION;
    Task1Control_log(control, control->endpoint2 ?
        "TASK1 END2 STARTED target=300 ramp=200ms\r\n" :
        "TASK1 END1 STARTED target=300 ramp=200ms\r\n");
    Stopwatch_start();
}

void Task1Control_init(Task1Control *control,
    const Task1Control_Config *config)
{
    control->config = *config;
    Task1Control_reset(control);
}

void Task1Control_reset(Task1Control *control)
{
    control->state = TASK1_CONTROL_WAIT_LOAD;
    control->intersectionPartialCount = 0U;
    control->advanceSamples = 0U;
    control->accelerationSamples = 0U;
    control->blankCount = 0U;
    control->brakeSamples = 0U;
    control->reverseSamples = 0U;
    control->finalBrakeSettledCount = 0U;
    control->lineSeenAfterTurn = false;
    control->endpoint2 = false;
    control->returnPhase = false;
    control->nextTurnRight = false;
    control->lastShowStopwatchSystick = systick_ms;
    control->config.setLineTrackingEnabled(false);
    control->config.resetLineTracking();
    AngleTurnControl_cancel(control->config.angleTurn);
    CarControl_stop(control->config.car);
}

bool Task1Control_isActive(const Task1Control *control)
{
    return (control->state != TASK1_CONTROL_WAIT_LOAD);
}

void Task1Control_update(Task1Control *control, TaskManager_Task activeTask,
    TaskManager_Task1Endpoint endpoint, bool startTaskPressed, bool mpu6050Ready,
    AngleTurnControl_Result angleTurnResult)
{
    uint8_t activeCount;

    if (activeTask != TASK_MANAGER_TASK_1) {
        if (Task1Control_isActive(control)) {
            Task1Control_reset(control);
        }
        return;
    }

    if (control->state == TASK1_CONTROL_WAIT_LOAD) {
        if (startTaskPressed) {
            Task1Control_start(control, endpoint);
        }
        return;
    }

    // 定时显示计时器
    if (systick_ms - control->lastShowStopwatchSystick >= 500)
    {
        uint64_t currentMs = Stopwatch_getCurrentMs();
        OLED_ShowStopwatch(currentMs);
        control->lastShowStopwatchSystick = systick_ms;
    }

    switch (control->state) {
        /* 去程或返程：加速巡线，直到确认目标十字路口。 */
        case TASK1_CONTROL_FOLLOW_INTERSECTION:
            Task1Control_applyRamp(control);
            if (Task1Control_detectIntersection(control)) {
                control->config.setLineTrackingEnabled(false);
                control->config.resetLineTracking();
                control->advanceSamples = 0U;
                control->nextTurnRight = control->returnPhase ?
                    !control->endpoint2 : control->endpoint2;
                control->state = TASK1_CONTROL_ADVANCE;
                CarControl_setMotion(control->config.car, CAR_CONTROL_FORWARD,
                    TASK1_CRUISE_TARGET_RPM, 100U);
                Task1Control_log(control, "TASK1 INTERSECTION ADVANCE 50ms\r\n");
            } else {
                control->config.updateLineTracking();
            }
            break;

        /* 保持直行一小段，使车体中心进入路口后再停车。 */
        case TASK1_CONTROL_ADVANCE:
            CarControl_setMotion(control->config.car, CAR_CONTROL_FORWARD,
                TASK1_CRUISE_TARGET_RPM, 100U);
            control->advanceSamples++;
            if (control->advanceSamples >= TASK1_ADVANCE_SAMPLES) {
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->state = TASK1_CONTROL_BRAKE_TURN;
                Task1Control_log(control, "TASK1 INTERSECTION BRAKING\r\n");
            }
            break;

        /* 等待车轮停稳，再启动 MPU6050 定角转向。 */
        case TASK1_CONTROL_BRAKE_TURN:
            control->brakeSamples++;
            if ((control->brakeSamples >= TASK1_BRAKE_MIN_SAMPLES) &&
                Task1Control_carStopped(control)) {
                if (mpu6050Ready && AngleTurnControl_start(control->config.angleTurn,
                        !control->nextTurnRight, TASK1_TURN_DEGREES,
                        TASK1_TURN_RPM)) {
                    control->state = TASK1_CONTROL_TURNING;
                    Task1Control_log(control, control->nextTurnRight ?
                        "TASK1 RIGHT TURN STARTED 85deg\r\n" :
                        "TASK1 LEFT TURN STARTED 85deg\r\n");
                } else {
                    CarControl_emergencyStop(control->config.car);
                    control->state = TASK1_CONTROL_FAULT;
                    Task1Control_log(control, "TASK1 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >= TASK1_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                Task1Control_log(control, "TASK1 BRAKE TIMEOUT\r\n");
            }
            break;

        /* 等待转向控制器完成；超时或反馈故障则进入 FAULT。 */
        case TASK1_CONTROL_TURNING:
            if (angleTurnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->config.setLineTrackingEnabled(true);
                control->lineSeenAfterTurn = false;
                control->blankCount = 0U;
                control->accelerationSamples = 0U;
                control->config.resetLineTracking();
                control->state = TASK1_CONTROL_FOLLOW_BLANK;
                Task1Control_log(control, "TASK1 TURN DONE, FOLLOWING\r\n");
            } else if ((angleTurnResult == ANGLE_TURN_RESULT_TIMEOUT) ||
                       (angleTurnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                Task1Control_log(control, "TASK1 TURN FAULT\r\n");
            }
            break;

        /* 转弯后继续巡线，连续检测到空白即认为到达终点。 */
        case TASK1_CONTROL_FOLLOW_BLANK:
            Task1Control_applyRamp(control);
            activeCount = control->config.readActiveChannelCount();
            if (activeCount > 0U) {
                control->lineSeenAfterTurn = true;
                control->blankCount = 0U;
            } else if (control->lineSeenAfterTurn) {
                if (control->blankCount < 255U) {
                    control->blankCount++;
                }
                if (control->blankCount >= TASK1_BLANK_CONFIRM_SAMPLES) {
                    control->config.setLineTrackingEnabled(false);
                    control->config.resetLineTracking();
                    CarControl_stop(control->config.car);
                    control->brakeSamples = 0U;
                    control->state = TASK1_CONTROL_END_BRAKE;
                    Task1Control_log(control, "TASK1 END BRAKING\r\n");
                    break;
                }
            }
            control->config.updateLineTracking();
            break;

        /* 到达终点后先主动制动，停稳后进入倒车微调。 */
        case TASK1_CONTROL_END_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >= TASK1_BRAKE_MIN_SAMPLES) &&
                Task1Control_carStopped(control)) {
                control->reverseSamples = 0U;
                CarControl_setMotion(control->config.car, CAR_CONTROL_BACKWARD,
                    TASK1_REVERSE_RPM, 100U);
                Stopwatch_stop();
            } else if (control->brakeSamples >= TASK1_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                Stopwatch_stop();
            }
            break;

        case TASK1_CONTROL_DONE:
        case TASK1_CONTROL_FAULT:
        default:
            break;
    }
}
