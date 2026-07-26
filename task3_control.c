#include "task3_control.h"

#define TASK3_OUTBOUND_CRUISE_RPM          (150)
#define TASK3_RETURN_CRUISE_RPM            (150)
#define TASK3_RAMP_SAMPLES                 (20U) /* 0.20 s */
#define TASK3_TARGET_INTERSECTION_NUMBER   (3U)
#define TASK3_INTERSECTION_THRESHOLD       (6U)
#define TASK3_INTERSECTION_CONFIRM_SAMPLES (2U)
#define TASK3_ADVANCE_SAMPLES              (16U) /* 0.16 s */
#define TASK3_MIN_BRAKE_SAMPLES            (15U)
#define TASK3_BRAKE_TIMEOUT_SAMPLES        (80U)
#define TASK3_TURN_DEGREES                 (85.0f)
#define TASK3_TURN_RPM                     (100)
#define TASK3_RETURN_DEGREES               (180.0f)
#define TASK3_REQUIRED_ROUTE_TURNS         (2U)
#define TASK3_LEFT_SENSOR_MASK             (0x0FU) /* X1..X4 */
#define TASK3_RIGHT_SENSOR_MASK            (0xF0U) /* X5..X8 */
#define TASK3_BLANK_CONFIRM_SAMPLES        (3U)
#define TASK3_REVERSE_RPM                  (75)
#define TASK3_REVERSE_SAMPLES              (50U) /* 0.50 s */
#define TASK3_FINAL_SETTLE_SAMPLES         (5U)

static void Task3Control_log(Task3Control *control, const char *text)
{
    if (control->io.log != 0) {
        control->io.log(text);
    }
}

static int16_t Task3Control_cruiseRpm(
    const Task3Control *control)
{
    return control->returnPhase ?
        TASK3_RETURN_CRUISE_RPM :
        TASK3_OUTBOUND_CRUISE_RPM;
}

static bool Task3Control_outboundTurnRight(
    TaskManager_Task3Endpoint endpoint, uint8_t turnIndex)
{
    if (turnIndex == 0U) {
        return (endpoint == TASK_MANAGER_TASK3_ENDPOINT_3) ||
            (endpoint == TASK_MANAGER_TASK3_ENDPOINT_4);
    }
    return (endpoint == TASK_MANAGER_TASK3_ENDPOINT_2) ||
        (endpoint == TASK_MANAGER_TASK3_ENDPOINT_4);
}

static bool Task3Control_carStopped(const Task3Control *control)
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

static void Task3Control_applyRamp(Task3Control *control)
{
    int16_t cruiseRpm = Task3Control_cruiseRpm(control);

    if (control->rampSamples < TASK3_RAMP_SAMPLES) {
        control->rampSamples++;
    }
    control->io.setLineTrackingSpeed((int16_t) (((int32_t)
        cruiseRpm * control->rampSamples) /
        TASK3_RAMP_SAMPLES));
}

static bool Task3Control_detectIntersection(
    Task3Control *control, uint8_t activeCount)
{
    if (activeCount == 8U) {
        control->intersectionConfirmCount = 0U;
        return true;
    }
    if (activeCount >= TASK3_INTERSECTION_THRESHOLD) {
        if (control->intersectionConfirmCount < 255U) {
            control->intersectionConfirmCount++;
        }
        return control->intersectionConfirmCount >=
            TASK3_INTERSECTION_CONFIRM_SAMPLES;
    }
    control->intersectionConfirmCount = 0U;
    return false;
}

static bool Task3Control_detectReturnDirection(
    uint8_t activeMask, bool *turnRight)
{
    bool leftDetected =
        (activeMask & TASK3_LEFT_SENSOR_MASK) ==
        TASK3_LEFT_SENSOR_MASK;
    bool rightDetected =
        (activeMask & TASK3_RIGHT_SENSOR_MASK) ==
        TASK3_RIGHT_SENSOR_MASK;

    /*
     * Exactly one side must be present. All eight sensors active is
     * directionally ambiguous, so keep following instead of guessing.
     */
    if (leftDetected == rightDetected) {
        return false;
    }

    *turnRight = rightDetected;
    return true;
}

static bool Task3Control_returnJunctionPresent(uint8_t activeMask)
{
    return (((activeMask & TASK3_LEFT_SENSOR_MASK) ==
                TASK3_LEFT_SENSOR_MASK) ||
            ((activeMask & TASK3_RIGHT_SENSOR_MASK) ==
                TASK3_RIGHT_SENSOR_MASK));
}

static void Task3Control_start(Task3Control *control,
    TaskManager_Task3Endpoint endpoint)
{
    control->endpoint = endpoint;
    control->outboundCrossCount = 0U;
    control->completedOutboundTurns = 0U;
    control->completedReturnTurns = 0U;
    control->intersectionConfirmCount = 0U;
    control->advanceSamples = 0U;
    control->rampSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
    control->returnPhase = false;
    control->nextTurnRight = false;
    control->intersectionArmed = true;
    control->lineSeenAfterTurn = false;
    control->io.setLineTrackingSpeed(0);
    control->io.setLineTrackingEnabled(true);
    control->io.resetLineTracking();
    AngleTurnControl_cancel(control->io.angleTurn);
    control->io.setRedLed(false);
    control->io.setGreenLed(false);
    control->state = TASK3_FOLLOW_THIRD_INTERSECTION;

    switch (endpoint) {
        case TASK_MANAGER_TASK3_ENDPOINT_1:
            Task3Control_log(control,
                "TASK3 END1 STARTED route=LEFT,LEFT\r\n");
            break;
        case TASK_MANAGER_TASK3_ENDPOINT_2:
            Task3Control_log(control,
                "TASK3 END2 STARTED route=LEFT,RIGHT\r\n");
            break;
        case TASK_MANAGER_TASK3_ENDPOINT_3:
            Task3Control_log(control,
                "TASK3 END3 STARTED route=RIGHT,LEFT\r\n");
            break;
        case TASK_MANAGER_TASK3_ENDPOINT_4:
        default:
            Task3Control_log(control,
                "TASK3 END4 STARTED route=RIGHT,RIGHT\r\n");
            break;
    }
}

static void Task3Control_beginAdvance(Task3Control *control)
{
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    control->advanceSamples = 0U;
    control->state = TASK3_ADVANCE;
    CarControl_setMotion(control->io.car, CAR_CONTROL_FORWARD,
        Task3Control_cruiseRpm(control), 100U);
}

static void Task3Control_beginEndBrake(Task3Control *control)
{
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    CarControl_stop(control->io.car);
    control->brakeSamples = 0U;
    control->state = TASK3_END_BRAKE;
}

static void Task3Control_prepareIntersectionFollow(
    Task3Control *control, Task3Control_State nextState)
{
    control->intersectionConfirmCount = 0U;
    control->intersectionArmed = false;
    control->rampSamples = 0U;
    control->blankSamples = 0U;
    control->lineSeenAfterTurn = false;
    control->io.setLineTrackingEnabled(true);
    control->io.resetLineTracking();
    control->state = nextState;
}

void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io)
{
    control->io = *io;
    Task3Control_reset(control);
}

void Task3Control_reset(Task3Control *control)
{
    control->state = TASK3_WAIT_LOAD;
    control->endpoint = TASK_MANAGER_TASK3_ENDPOINT_1;
    control->outboundCrossCount = 0U;
    control->completedOutboundTurns = 0U;
    control->completedReturnTurns = 0U;
    control->intersectionConfirmCount = 0U;
    control->advanceSamples = 0U;
    control->rampSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
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

bool Task3Control_isActive(const Task3Control *control)
{
    return control->state != TASK3_WAIT_LOAD;
}

void Task3Control_update(Task3Control *control,
    TaskManager_Task activeTask, TaskManager_Task3Endpoint endpoint,
    bool statusPressed, bool statusReleased, bool mpuReady,
    AngleTurnControl_Result turnResult)
{
    uint8_t activeCount;
    uint8_t activeMask;

    if (activeTask != TASK_MANAGER_TASK_3) {
        if (Task3Control_isActive(control)) {
            Task3Control_reset(control);
        }
        return;
    }

    if (control->state == TASK3_WAIT_LOAD) {
        if (statusPressed) {
            Task3Control_start(control, endpoint);
        }
        return;
    }

    switch (control->state) {
        case TASK3_FOLLOW_THIRD_INTERSECTION:
            Task3Control_applyRamp(control);
            activeCount =
                control->io.readActiveChannelCount();

            if (activeCount < TASK3_INTERSECTION_THRESHOLD) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task3Control_detectIntersection(
                    control, activeCount)) {
                control->intersectionArmed = false;
                control->outboundCrossCount++;

                if (control->outboundCrossCount >=
                    TASK3_TARGET_INTERSECTION_NUMBER) {
                    control->nextTurnRight =
                        Task3Control_outboundTurnRight(
                            control->endpoint, 0U);
                    Task3Control_beginAdvance(control);
                    Task3Control_log(control,
                        "TASK3 CROSS3 ADVANCE 160ms\r\n");
                    break;
                }
                Task3Control_log(control,
                    (control->outboundCrossCount == 1U) ?
                    "TASK3 CROSS1 PASSED STRAIGHT\r\n" :
                    "TASK3 CROSS2 PASSED STRAIGHT\r\n");
            }

            control->io.updateLineTracking();
            break;

        case TASK3_FOLLOW_NEXT_INTERSECTION:
            Task3Control_applyRamp(control);
            activeCount =
                control->io.readActiveChannelCount();

            if (activeCount < TASK3_INTERSECTION_THRESHOLD) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task3Control_detectIntersection(
                    control, activeCount)) {
                control->intersectionArmed = false;
                control->nextTurnRight =
                    Task3Control_outboundTurnRight(
                        control->endpoint, 1U);
                Task3Control_beginAdvance(control);
                Task3Control_log(control,
                    "TASK3 NEXT CROSS ADVANCE 160ms\r\n");
                break;
            }

            control->io.updateLineTracking();
            break;

        case TASK3_RETURN_INTERSECTION:
            Task3Control_applyRamp(control);
            activeMask =
                control->io.readActiveChannelMask();

            /*
             * A completed turn starts disarmed. The car must first leave
             * the old junction before a new left/right half can trigger.
             */
            if (!Task3Control_returnJunctionPresent(activeMask)) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task3Control_detectReturnDirection(
                    activeMask, &control->nextTurnRight)) {
                control->intersectionArmed = false;
                Task3Control_beginAdvance(control);
                Task3Control_log(control,
                    control->nextTurnRight ?
                    "TASK3 RETURN RIGHT HALF ADVANCE 160ms\r\n" :
                    "TASK3 RETURN LEFT HALF ADVANCE 160ms\r\n");
                break;
            }

            control->io.updateLineTracking();
            break;

        case TASK3_ADVANCE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_FORWARD,
                Task3Control_cruiseRpm(control), 100U);
            control->advanceSamples++;
            if (control->advanceSamples >=
                TASK3_ADVANCE_SAMPLES) {
                CarControl_stop(control->io.car);
                control->brakeSamples = 0U;
                control->state = TASK3_BRAKE_TURN;
                Task3Control_log(control,
                    "TASK3 TURN PID BRAKING\r\n");
            }
            break;

        case TASK3_BRAKE_TURN:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK3_MIN_BRAKE_SAMPLES) &&
                Task3Control_carStopped(control)) {
                bool turnLeft = !control->nextTurnRight;

                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        turnLeft, TASK3_TURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->state = TASK3_TURNING;
                    Task3Control_log(control, turnLeft ?
                        "TASK3 LEFT TURN STARTED 85deg\r\n" :
                        "TASK3 RIGHT TURN STARTED 85deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
                    control->io.setRedLed(false);
                    Task3Control_log(control,
                        "TASK3 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >=
                TASK3_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(false);
                Task3Control_log(control,
                    "TASK3 TURN BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_TURNING:
            if (turnResult == ANGLE_TURN_RESULT_COMPLETED) {
                if (control->returnPhase) {
                    control->completedReturnTurns++;
                    if (control->completedReturnTurns >=
                        TASK3_REQUIRED_ROUTE_TURNS) {
                        Task3Control_prepareIntersectionFollow(
                            control, TASK3_FOLLOW_BLANK);
                        Task3Control_log(control,
                            "TASK3 RETURN TURN2 DONE, FOLLOW START\r\n");
                    } else {
                        Task3Control_prepareIntersectionFollow(
                            control, TASK3_RETURN_INTERSECTION);
                        Task3Control_log(control,
                            "TASK3 RETURN TURN1 DONE\r\n");
                    }
                } else {
                    control->completedOutboundTurns++;
                    if (control->completedOutboundTurns >=
                        TASK3_REQUIRED_ROUTE_TURNS) {
                        Task3Control_prepareIntersectionFollow(
                            control, TASK3_FOLLOW_BLANK);
                        Task3Control_log(control,
                            "TASK3 OUTBOUND TURN2 DONE, FOLLOW END\r\n");
                    } else {
                        Task3Control_prepareIntersectionFollow(
                            control, TASK3_FOLLOW_NEXT_INTERSECTION);
                        Task3Control_log(control,
                            "TASK3 OUTBOUND TURN1 DONE\r\n");
                    }
                }
            } else if ((turnResult ==
                    ANGLE_TURN_RESULT_TIMEOUT) ||
                (turnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(false);
                Task3Control_log(control,
                    "TASK3 TURN FAULT\r\n");
            }
            break;

        case TASK3_FOLLOW_BLANK:
            Task3Control_applyRamp(control);
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
                    TASK3_BLANK_CONFIRM_SAMPLES) {
                    Task3Control_beginEndBrake(control);
                    Task3Control_log(control,
                        control->returnPhase ?
                        "TASK3 RETURN END BRAKING\r\n" :
                        "TASK3 DESTINATION BRAKING\r\n");
                    break;
                }
            }

            control->io.updateLineTracking();
            break;

        case TASK3_END_BRAKE:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK3_MIN_BRAKE_SAMPLES) &&
                Task3Control_carStopped(control)) {
                control->reverseSamples = 0U;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_BACKWARD,
                    TASK3_REVERSE_RPM, 100U);
                control->state = TASK3_REVERSE;
                Task3Control_log(control,
                    "TASK3 REVERSE 75RPM 500ms\r\n");
            } else if (control->brakeSamples >=
                TASK3_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(false);
                Task3Control_log(control,
                    "TASK3 END BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_REVERSE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_BACKWARD,
                TASK3_REVERSE_RPM, 100U);
            control->reverseSamples++;
            if (control->reverseSamples >=
                TASK3_REVERSE_SAMPLES) {
                CarControl_stop(control->io.car);
                control->brakeSamples = 0U;
                control->settledSamples = 0U;
                control->state = TASK3_FINAL_BRAKE;
                Task3Control_log(control,
                    "TASK3 FINAL PID BRAKING\r\n");
            }
            break;

        case TASK3_FINAL_BRAKE:
            control->brakeSamples++;
            if (Task3Control_carStopped(control)) {
                if (control->settledSamples < 255U) {
                    control->settledSamples++;
                }
                if (control->settledSamples >=
                    TASK3_FINAL_SETTLE_SAMPLES) {
                    CarControl_emergencyStop(control->io.car);
                    if (control->returnPhase) {
                        control->state = TASK3_DONE;
                        control->io.setRedLed(false);
                        control->io.setGreenLed(true);
                        Task3Control_log(control,
                            "TASK3 RETURN DONE\r\n");
                    } else {
                        control->state = TASK3_WAIT_UNLOAD;
                        control->io.setRedLed(true);
                        control->io.setGreenLed(false);
                        Task3Control_log(control,
                            "TASK3 WAIT UNLOAD\r\n");
                    }
                }
            } else {
                control->settledSamples = 0U;
            }

            if ((control->state == TASK3_FINAL_BRAKE) &&
                (control->brakeSamples >=
                    TASK3_BRAKE_TIMEOUT_SAMPLES)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(false);
                Task3Control_log(control,
                    "TASK3 FINAL BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_WAIT_UNLOAD:
            if (statusReleased) {
                bool finalOutboundTurnRight =
                    Task3Control_outboundTurnRight(
                        control->endpoint, 1U);

                control->io.setRedLed(false);
                control->io.setGreenLed(false);
                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        !finalOutboundTurnRight,
                        TASK3_RETURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->state = TASK3_RETURN_TURNING;
                    Task3Control_log(control,
                        "TASK3 RETURN TURN STARTED 180deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
                    control->io.setRedLed(false);
                    Task3Control_log(control,
                        "TASK3 RETURN TURN START FAILED\r\n");
                }
            }
            break;

        case TASK3_RETURN_TURNING:
            if (turnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->returnPhase = true;
                control->completedReturnTurns = 0U;
                Task3Control_prepareIntersectionFollow(control,
                    TASK3_RETURN_INTERSECTION);
                Task3Control_log(control,
                    "TASK3 RETURN FOLLOWING\r\n");
            } else if ((turnResult ==
                    ANGLE_TURN_RESULT_TIMEOUT) ||
                (turnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(false);
                Task3Control_log(control,
                    "TASK3 RETURN TURN FAULT\r\n");
            }
            break;

        case TASK3_DONE:
        case TASK3_FAULT:
        default:
            break;
    }
}
