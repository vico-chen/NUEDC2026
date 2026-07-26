#include "task1_control.h"

#define TASK1_CRUISE_TARGET_RPM (150)
#define TASK1_ACCELERATION_SAMPLES (20U)
#define TASK1_INTERSECTION_THRESHOLD (6U)
#define TASK1_INTERSECTION_CONFIRM_SAMPLES (2U)
#define TASK1_ADVANCE_SAMPLES (16U) /* 0.16 s */
#define TASK1_BRAKE_MIN_SAMPLES (15U)
#define TASK1_BRAKE_TIMEOUT_SAMPLES (80U)
#define TASK1_TURN_DEGREES (85.0f)
#define TASK1_TURN_RPM (100)
#define TASK1_RETURN_DEGREES (180.0f)
#define TASK1_BLANK_CONFIRM_SAMPLES (3U)
#define TASK1_REVERSE_RPM (75)
#define TASK1_REVERSE_SAMPLES (50U)
#define TASK1_FINAL_SETTLE_SAMPLES (5U)

static void Task1Control_log(Task1Control *control, const char *text)
{
    if (control->config.log != 0) {
        control->config.log(text);
    }
}

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

static void Task1Control_applyRamp(Task1Control *control)
{
    if (control->accelerationSamples < TASK1_ACCELERATION_SAMPLES) {
        control->accelerationSamples++;
    }
    control->config.setLineTrackingSpeed((int16_t) (((int32_t)
        TASK1_CRUISE_TARGET_RPM * control->accelerationSamples) /
        TASK1_ACCELERATION_SAMPLES));
}

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
    control->config.setRedLed(false);
    control->config.setGreenLed(false);
    control->state = TASK1_CONTROL_FOLLOW_INTERSECTION;
    Task1Control_log(control, control->endpoint2 ?
        "TASK1 END2 STARTED target=150 ramp=200ms\r\n" :
        "TASK1 END1 STARTED target=150 ramp=200ms\r\n");
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
    control->config.setLineTrackingEnabled(false);
    control->config.resetLineTracking();
    AngleTurnControl_cancel(control->config.angleTurn);
    CarControl_stop(control->config.car);
    control->config.setRedLed(false);
    control->config.setGreenLed(false);
}

bool Task1Control_isActive(const Task1Control *control)
{
    return (control->state != TASK1_CONTROL_WAIT_LOAD);
}

void Task1Control_update(Task1Control *control, TaskManager_Task activeTask,
    TaskManager_Task1Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool mpu6050Ready,
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
        if (statusPressed) {
            Task1Control_start(control, endpoint);
        }
        return;
    }

    switch (control->state) {
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
                Task1Control_log(control, "TASK1 INTERSECTION ADVANCE 160ms\r\n");
            } else {
                control->config.updateLineTracking();
            }
            break;

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
                    control->config.setRedLed(false);
                    Task1Control_log(control, "TASK1 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >= TASK1_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                control->config.setRedLed(false);
                Task1Control_log(control, "TASK1 BRAKE TIMEOUT\r\n");
            }
            break;

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
                control->config.setRedLed(false);
                Task1Control_log(control, "TASK1 TURN FAULT\r\n");
            }
            break;

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

        case TASK1_CONTROL_END_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >= TASK1_BRAKE_MIN_SAMPLES) &&
                Task1Control_carStopped(control)) {
                control->reverseSamples = 0U;
                CarControl_setMotion(control->config.car, CAR_CONTROL_BACKWARD,
                    TASK1_REVERSE_RPM, 100U);
                control->state = TASK1_CONTROL_END_REVERSE;
                Task1Control_log(control, "TASK1 REVERSE 75RPM 500ms\r\n");
            } else if (control->brakeSamples >= TASK1_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                control->config.setRedLed(false);
                Task1Control_log(control, "TASK1 END BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK1_CONTROL_END_REVERSE:
            CarControl_setMotion(control->config.car, CAR_CONTROL_BACKWARD,
                TASK1_REVERSE_RPM, 100U);
            control->reverseSamples++;
            if (control->reverseSamples >= TASK1_REVERSE_SAMPLES) {
                CarControl_stop(control->config.car);
                control->brakeSamples = 0U;
                control->finalBrakeSettledCount = 0U;
                control->state = TASK1_CONTROL_FINAL_BRAKE;
                Task1Control_log(control, "TASK1 FINAL PID BRAKING\r\n");
            }
            break;

        case TASK1_CONTROL_FINAL_BRAKE:
            control->brakeSamples++;
            if (Task1Control_carStopped(control)) {
                if (control->finalBrakeSettledCount < 255U) {
                    control->finalBrakeSettledCount++;
                }
                if (control->finalBrakeSettledCount >= TASK1_FINAL_SETTLE_SAMPLES) {
                    CarControl_emergencyStop(control->config.car);
                    if (control->returnPhase) {
                        control->state = TASK1_CONTROL_DONE;
                        control->config.setRedLed(false);
                        control->config.setGreenLed(true);
                        Task1Control_log(control, "TASK1 RETURN DONE\r\n");
                    } else {
                        control->state = TASK1_CONTROL_WAIT_UNLOAD;
                        control->config.setRedLed(true);
                        control->config.setGreenLed(false);
                        Task1Control_log(control, "TASK1 WAIT UNLOAD\r\n");
                    }
                }
            } else {
                control->finalBrakeSettledCount = 0U;
            }
            if ((control->state == TASK1_CONTROL_FINAL_BRAKE) &&
                (control->brakeSamples >= TASK1_BRAKE_TIMEOUT_SAMPLES)) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                control->config.setRedLed(false);
                Task1Control_log(control, "TASK1 FINAL BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK1_CONTROL_WAIT_UNLOAD:
            if (statusReleased) {
                control->config.setRedLed(false);
                control->config.setGreenLed(false);
                if (mpu6050Ready && AngleTurnControl_start(control->config.angleTurn,
                        !control->endpoint2, TASK1_RETURN_DEGREES,
                        TASK1_TURN_RPM)) {
                    control->state = TASK1_CONTROL_RETURN_TURNING;
                    Task1Control_log(control, "TASK1 RETURN TURN STARTED 180deg\r\n");
                } else {
                    CarControl_emergencyStop(control->config.car);
                    control->state = TASK1_CONTROL_FAULT;
                    control->config.setRedLed(false);
                    Task1Control_log(control, "TASK1 RETURN TURN START FAILED\r\n");
                }
            }
            break;

        case TASK1_CONTROL_RETURN_TURNING:
            if (angleTurnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->returnPhase = true;
                control->config.setLineTrackingEnabled(true);
                control->lineSeenAfterTurn = false;
                control->intersectionPartialCount = 0U;
                control->blankCount = 0U;
                control->accelerationSamples = 0U;
                control->config.resetLineTracking();
                control->state = TASK1_CONTROL_FOLLOW_INTERSECTION;
                Task1Control_log(control, "TASK1 RETURN FOLLOWING\r\n");
            } else if ((angleTurnResult == ANGLE_TURN_RESULT_TIMEOUT) ||
                       (angleTurnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->config.car);
                control->state = TASK1_CONTROL_FAULT;
                control->config.setRedLed(false);
                Task1Control_log(control, "TASK1 RETURN TURN FAULT\r\n");
            }
            break;

        case TASK1_CONTROL_DONE:
        case TASK1_CONTROL_FAULT:
        default:
            break;
    }
}
