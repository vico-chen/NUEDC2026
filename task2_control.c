#include "task2_control.h"

#define TASK2_CRUISE_RPM                    (90)
#define TASK2_RAMP_SAMPLES                  (20U) /* 0.20 s */
#define TASK2_TARGET_INTERSECTION_NUMBER    (2U)
#define TASK2_INTERSECTION_THRESHOLD        (6U)
#define TASK2_INTERSECTION_CONFIRM_SAMPLES  (2U)
#define TASK2_ADVANCE_SAMPLES               (22U) /* 0.22 s */
#define TASK2_MIN_BRAKE_SAMPLES             (15U)
#define TASK2_BRAKE_TIMEOUT_SAMPLES         (80U)
#define TASK2_TURN_DEGREES                  (85.0f)
#define TASK2_TURN_RPM                      (100)
#define TASK2_RETURN_DEGREES                (180.0f)
#define TASK2_BLANK_CONFIRM_SAMPLES         (3U)
#define TASK2_REVERSE_RPM                   (75)
#define TASK2_REVERSE_SAMPLES               (50U) /* 0.50 s */
#define TASK2_FINAL_SETTLE_SAMPLES          (5U)

static void Task2Control_log(Task2Control *control, const char *text)
{
    if (control->io.log != 0) {
        control->io.log(text);
    }
}

static bool Task2Control_carStopped(const Task2Control *control)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        int32_t counts =
            control->io.motors[i]->speedCountsPerSample;
        int32_t deadband =
            control->io.motors[i]->config.zeroSpeedDeadbandCounts;

        if ((counts > deadband) || (counts < -deadband)) {
            return false;
        }
    }
    return true;
}

static void Task2Control_applyRamp(Task2Control *control)
{
    if (control->rampSamples < TASK2_RAMP_SAMPLES) {
        control->rampSamples++;
    }
    control->io.setLineTrackingSpeed((int16_t) (((int32_t)
        TASK2_CRUISE_RPM * control->rampSamples) /
        TASK2_RAMP_SAMPLES));
}

static bool Task2Control_detectIntersection(
    Task2Control *control, uint8_t activeCount)
{
    if (activeCount == 8U) {
        control->intersectionConfirmCount = 0U;
        return true;
    }
    if (activeCount >= TASK2_INTERSECTION_THRESHOLD) {
        if (control->intersectionConfirmCount < 255U) {
            control->intersectionConfirmCount++;
        }
        return control->intersectionConfirmCount >=
            TASK2_INTERSECTION_CONFIRM_SAMPLES;
    }
    control->intersectionConfirmCount = 0U;
    return false;
}

static void Task2Control_start(Task2Control *control,
    TaskManager_Task1Endpoint endpoint)
{
    control->endpoint2 =
        (endpoint == TASK_MANAGER_TASK1_ENDPOINT_2);
    control->returnPhase = false;
    control->nextTurnRight = false;
    control->outboundCrossCount = 0U;
    control->intersectionConfirmCount = 0U;
    control->advanceSamples = 0U;
    control->rampSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
    control->intersectionArmed = true;
    control->lineSeenAfterTurn = false;
    control->io.setLineTrackingSpeed(0);
    control->io.setLineTrackingEnabled(true);
    control->io.resetLineTracking();
    AngleTurnControl_cancel(control->io.angleTurn);
    control->io.setRedLed(false);
    control->io.setGreenLed(false);
    control->state = TASK2_FOLLOW_SECOND_INTERSECTION;
    Task2Control_log(control, control->endpoint2 ?
        "TASK2 END2 STARTED, TURN AT CROSS2\r\n" :
        "TASK2 END1 STARTED, TURN AT CROSS2\r\n");
}

static void Task2Control_beginAdvance(Task2Control *control)
{
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    control->advanceSamples = 0U;
    control->state = TASK2_ADVANCE;
    CarControl_setMotion(control->io.car, CAR_CONTROL_FORWARD,
        TASK2_CRUISE_RPM, 100U);
}

static void Task2Control_beginEndBrake(Task2Control *control)
{
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    CarControl_stop(control->io.car);
    control->brakeSamples = 0U;
    control->state = TASK2_END_BRAKE;
}

void Task2Control_init(Task2Control *control,
    const Task1Control_Config *io)
{
    control->io = *io;
    Task2Control_reset(control);
}

void Task2Control_reset(Task2Control *control)
{
    control->state = TASK2_WAIT_LOAD;
    control->outboundCrossCount = 0U;
    control->intersectionConfirmCount = 0U;
    control->advanceSamples = 0U;
    control->rampSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
    control->endpoint2 = false;
    control->returnPhase = false;
    control->nextTurnRight = false;
    control->intersectionArmed = false;
    control->lineSeenAfterTurn = false;
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    AngleTurnControl_cancel(control->io.angleTurn);
    CarControl_stop(control->io.car);
    control->io.setRedLed(false);
    control->io.setGreenLed(false);
}

bool Task2Control_isActive(const Task2Control *control)
{
    return control->state != TASK2_WAIT_LOAD;
}

void Task2Control_update(Task2Control *control,
    TaskManager_Task activeTask, TaskManager_Task1Endpoint endpoint,
    bool statusPressed, bool statusReleased, bool mpuReady,
    AngleTurnControl_Result turnResult)
{
    uint8_t activeCount;

    if (activeTask != TASK_MANAGER_TASK_2) {
        if (Task2Control_isActive(control)) {
            Task2Control_reset(control);
        }
        return;
    }

    if (control->state == TASK2_WAIT_LOAD) {
        if (statusPressed) {
            Task2Control_start(control, endpoint);
        }
        return;
    }

    switch (control->state) {
        case TASK2_FOLLOW_SECOND_INTERSECTION:
            Task2Control_applyRamp(control);
            activeCount =
                control->io.readActiveChannelCount();

            if (activeCount < TASK2_INTERSECTION_THRESHOLD) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task2Control_detectIntersection(
                    control, activeCount)) {
                control->intersectionArmed = false;
                control->outboundCrossCount++;

                if (control->outboundCrossCount >=
                    TASK2_TARGET_INTERSECTION_NUMBER) {
                    control->nextTurnRight = control->endpoint2;
                    Task2Control_beginAdvance(control);
                    Task2Control_log(control,
                        "TASK2 CROSS2 ADVANCE 220ms\r\n");
                    break;
                }
                Task2Control_log(control,
                    "TASK2 CROSS1 PASSED STRAIGHT\r\n");
            }

            control->io.updateLineTracking();
            break;

        case TASK2_RETURN_INTERSECTION:
            Task2Control_applyRamp(control);
            activeCount =
                control->io.readActiveChannelCount();

            if (activeCount < TASK2_INTERSECTION_THRESHOLD) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task2Control_detectIntersection(
                    control, activeCount)) {
                control->intersectionArmed = false;
                control->nextTurnRight = !control->endpoint2;
                Task2Control_beginAdvance(control);
                Task2Control_log(control,
                    "TASK2 RETURN CROSS ADVANCE 220ms\r\n");
                break;
            }

            control->io.updateLineTracking();
            break;

        case TASK2_ADVANCE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_FORWARD, TASK2_CRUISE_RPM, 100U);
            control->advanceSamples++;
            if (control->advanceSamples >=
                TASK2_ADVANCE_SAMPLES) {
                CarControl_stop(control->io.car);
                control->brakeSamples = 0U;
                control->state = TASK2_BRAKE_TURN;
                Task2Control_log(control,
                    "TASK2 TURN PID BRAKING\r\n");
            }
            break;

        case TASK2_BRAKE_TURN:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK2_MIN_BRAKE_SAMPLES) &&
                Task2Control_carStopped(control)) {
                bool turnLeft = !control->nextTurnRight;

                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        turnLeft, TASK2_TURN_DEGREES,
                        TASK2_TURN_RPM)) {
                    control->state = TASK2_TURNING;
                    Task2Control_log(control, turnLeft ?
                        "TASK2 LEFT TURN STARTED 85deg\r\n" :
                        "TASK2 RIGHT TURN STARTED 85deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK2_FAULT;
                    control->io.setRedLed(true);
                    Task2Control_log(control,
                        "TASK2 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >=
                TASK2_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK2_FAULT;
                control->io.setRedLed(true);
                Task2Control_log(control,
                    "TASK2 TURN BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK2_TURNING:
            if (turnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->rampSamples = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterTurn = false;
                control->intersectionConfirmCount = 0U;
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(true);
                control->io.resetLineTracking();
                control->state = TASK2_FOLLOW_BLANK;
                Task2Control_log(control, control->returnPhase ?
                    "TASK2 RETURN TURN DONE, FOLLOW START\r\n" :
                    "TASK2 OUTBOUND TURN DONE, FOLLOW END\r\n");
            } else if ((turnResult ==
                    ANGLE_TURN_RESULT_TIMEOUT) ||
                (turnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK2_FAULT;
                control->io.setRedLed(true);
                Task2Control_log(control,
                    "TASK2 TURN FAULT\r\n");
            }
            break;

        case TASK2_FOLLOW_BLANK:
            Task2Control_applyRamp(control);
            activeCount =
                control->io.readActiveChannelCount();

            if (activeCount > 0U) {
                control->lineSeenAfterTurn = true;
                control->blankSamples = 0U;
            } else if (control->lineSeenAfterTurn) {
                if (control->blankSamples < 255U) {
                    control->blankSamples++;
                }
                if (control->blankSamples >=
                    TASK2_BLANK_CONFIRM_SAMPLES) {
                    Task2Control_beginEndBrake(control);
                    Task2Control_log(control,
                        control->returnPhase ?
                        "TASK2 RETURN END BRAKING\r\n" :
                        "TASK2 DESTINATION BRAKING\r\n");
                    break;
                }
            }

            control->io.updateLineTracking();
            break;

        case TASK2_END_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK2_MIN_BRAKE_SAMPLES) &&
                Task2Control_carStopped(control)) {
                control->reverseSamples = 0U;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_BACKWARD,
                    TASK2_REVERSE_RPM, 100U);
                control->state = TASK2_REVERSE;
                Task2Control_log(control,
                    "TASK2 REVERSE 75RPM 500ms\r\n");
            } else if (control->brakeSamples >=
                TASK2_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK2_FAULT;
                control->io.setRedLed(true);
                Task2Control_log(control,
                    "TASK2 END BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK2_REVERSE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_BACKWARD,
                TASK2_REVERSE_RPM, 100U);
            control->reverseSamples++;
            if (control->reverseSamples >=
                TASK2_REVERSE_SAMPLES) {
                CarControl_stop(control->io.car);
                control->brakeSamples = 0U;
                control->settledSamples = 0U;
                control->state = TASK2_FINAL_BRAKE;
                Task2Control_log(control,
                    "TASK2 FINAL PID BRAKING\r\n");
            }
            break;

        case TASK2_FINAL_BRAKE:
            control->brakeSamples++;
            if (Task2Control_carStopped(control)) {
                if (control->settledSamples < 255U) {
                    control->settledSamples++;
                }
                if (control->settledSamples >=
                    TASK2_FINAL_SETTLE_SAMPLES) {
                    CarControl_emergencyStop(control->io.car);
                    if (control->returnPhase) {
                        control->state = TASK2_DONE;
                        control->io.setRedLed(false);
                        control->io.setGreenLed(true);
                        Task2Control_log(control,
                            "TASK2 RETURN DONE\r\n");
                    } else {
                        control->state = TASK2_WAIT_UNLOAD;
                        control->io.setRedLed(true);
                        control->io.setGreenLed(false);
                        Task2Control_log(control,
                            "TASK2 WAIT UNLOAD\r\n");
                    }
                }
            } else {
                control->settledSamples = 0U;
            }

            if ((control->state == TASK2_FINAL_BRAKE) &&
                (control->brakeSamples >=
                    TASK2_BRAKE_TIMEOUT_SAMPLES)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK2_FAULT;
                control->io.setRedLed(true);
                Task2Control_log(control,
                    "TASK2 FINAL BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK2_WAIT_UNLOAD:
            if (statusReleased) {
                control->io.setRedLed(false);
                control->io.setGreenLed(false);
                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        !control->endpoint2,
                        TASK2_RETURN_DEGREES,
                        TASK2_TURN_RPM)) {
                    control->state = TASK2_RETURN_TURNING;
                    Task2Control_log(control,
                        "TASK2 RETURN TURN STARTED 180deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK2_FAULT;
                    control->io.setRedLed(true);
                    Task2Control_log(control,
                        "TASK2 RETURN TURN START FAILED\r\n");
                }
            }
            break;

        case TASK2_RETURN_TURNING:
            if (turnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->returnPhase = true;
                control->intersectionConfirmCount = 0U;
                control->intersectionArmed = false;
                control->rampSamples = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterTurn = false;
                control->io.setLineTrackingEnabled(true);
                control->io.resetLineTracking();
                control->state = TASK2_RETURN_INTERSECTION;
                Task2Control_log(control,
                    "TASK2 RETURN FOLLOWING\r\n");
            } else if ((turnResult ==
                    ANGLE_TURN_RESULT_TIMEOUT) ||
                (turnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK2_FAULT;
                control->io.setRedLed(true);
                Task2Control_log(control,
                    "TASK2 RETURN TURN FAULT\r\n");
            }
            break;

        case TASK2_DONE:
        case TASK2_FAULT:
        default:
            break;
    }
}
