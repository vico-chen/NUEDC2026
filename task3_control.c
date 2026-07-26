#include "task3_control.h"

#define TASK3_OUTBOUND_CRUISE_RPM         (50)
#define TASK3_RETURN_CRUISE_RPM           (110)
#define TASK3_RAMP_SAMPLES                (20U) /* 0.20 s */
#define TASK3_INTERSECTION_THRESHOLD      (6U)
#define TASK3_INTERSECTION_CONFIRM        (2U)
#define TASK3_ADVANCE_SAMPLES             (22U) /* 0.22 s */
#define TASK3_MIN_BRAKE_SAMPLES           (15U)
#define TASK3_BRAKE_TIMEOUT_SAMPLES       (80U)
#define TASK3_TURN_DEGREES                (85.0f)
#define TASK3_TURN_RPM                    (100)
#define TASK3_REQUIRED_TURNS              (2U)
#define TASK3_RETURN_TURN_DEGREES         (180.0f)
#define TASK3_LEFT_SENSOR_MASK            (0x0FU) /* X1..X4 */
#define TASK3_RIGHT_SENSOR_MASK           (0xF0U) /* X5..X8 */
#define TASK3_BLANK_CONFIRM_SAMPLES       (3U)
#define TASK3_REVERSE_RPM                 (75)
#define TASK3_REVERSE_SAMPLES             (50U) /* 0.50 s */
#define TASK3_FINAL_SETTLE_SAMPLES        (5U)

static void Task3Control_log(Task3Control *control, const char *text)
{
    if (control->io.log != 0) {
        control->io.log(text);
    }
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

static void Task3Control_followLine(Task3Control *control)
{
    int16_t cruiseRpm = control->returning ?
        TASK3_RETURN_CRUISE_RPM : TASK3_OUTBOUND_CRUISE_RPM;

    if (control->rampSamples < TASK3_RAMP_SAMPLES) {
        control->rampSamples++;
    }
    control->io.setLineTrackingSpeed((int16_t) (((int32_t)
        cruiseRpm * control->rampSamples) /
        TASK3_RAMP_SAMPLES));
    control->io.updateLineTracking();
}

static bool Task3Control_detectIntersection(
    Task3Control *control, uint8_t activeCount)
{
    if (activeCount == 8U) {
        control->intersectionConfirmSamples = 0U;
        return true;
    }
    if (activeCount >= TASK3_INTERSECTION_THRESHOLD) {
        if (control->intersectionConfirmSamples < 255U) {
            control->intersectionConfirmSamples++;
        }
        return control->intersectionConfirmSamples >=
            TASK3_INTERSECTION_CONFIRM;
    }
    control->intersectionConfirmSamples = 0U;
    return false;
}

static Task3Control_Turn Task3Control_detectReturnJunction(
    uint8_t activeMask)
{
    bool leftDetected =
        (activeMask & TASK3_LEFT_SENSOR_MASK) ==
        TASK3_LEFT_SENSOR_MASK;
    bool rightDetected =
        (activeMask & TASK3_RIGHT_SENSOR_MASK) ==
        TASK3_RIGHT_SENSOR_MASK;

    /*
     * Exactly one half must identify the T junction. If all eight channels
     * are active, the direction is ambiguous and the car keeps following.
     */
    if (leftDetected == rightDetected) {
        return TASK3_TURN_NONE;
    }
    return leftDetected ? TASK3_TURN_LEFT : TASK3_TURN_RIGHT;
}

static bool Task3Control_returnJunctionPresent(uint8_t activeMask)
{
    return (((activeMask & TASK3_LEFT_SENSOR_MASK) ==
                TASK3_LEFT_SENSOR_MASK) ||
            ((activeMask & TASK3_RIGHT_SENSOR_MASK) ==
                TASK3_RIGHT_SENSOR_MASK));
}

static uint8_t Task3Control_countActiveChannels(uint8_t activeMask)
{
    uint8_t activeCount = 0U;

    while (activeMask != 0U) {
        activeCount = (uint8_t) (activeCount +
            (activeMask & 0x01U));
        activeMask >>= 1U;
    }
    return activeCount;
}

static void Task3Control_beginPidBrake(Task3Control *control,
    Task3Control_State nextState)
{
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    CarControl_stop(control->io.car);
    control->brakeSamples = 0U;
    control->state = nextState;
}

void Task3Control_reset(Task3Control *control)
{
    control->state = TASK3_WAIT_INFO;
    control->pendingTurn = TASK3_TURN_NONE;
    control->completedTurns = 0U;
    control->completedReturnTurns = 0U;
    control->rampSamples = 0U;
    control->intersectionConfirmSamples = 0U;
    control->advanceSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
    control->lineSeenAfterFinalTurn = false;
    control->intersectionArmed = false;
    control->returning = false;
    control->uTurn = false;
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    AngleTurnControl_cancel(control->io.angleTurn);
    CarControl_stop(control->io.car);
    control->io.setRedLed(false);
    control->io.setGreenLed(false);
}

void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io)
{
    control->io = *io;
    Task3Control_reset(control);
}

void Task3Control_update(Task3Control *control, TaskManager_Task task,
    bool statusPressed, bool statusReleased, bool numberReceived,
    Task3Control_Turn visualTurn, bool mpuReady,
    AngleTurnControl_Result turnResult)
{
    uint8_t activeCount;
    uint8_t activeMask;

    if (task != TASK_MANAGER_TASK_3) {
        if (control->state != TASK3_WAIT_INFO) {
            Task3Control_reset(control);
        }
        return;
    }

    if (numberReceived && (control->state == TASK3_WAIT_INFO)) {
        control->state = TASK3_WAIT_LOAD;
        Task3Control_log(control, "TASK3 VISION NUMBER READY\r\n");
    }

    if ((visualTurn != TASK3_TURN_NONE) &&
        (control->state == TASK3_FOLLOW) &&
        (control->completedTurns < TASK3_REQUIRED_TURNS) &&
        control->intersectionArmed &&
        (control->pendingTurn == TASK3_TURN_NONE)) {
        control->pendingTurn = visualTurn;
        Task3Control_log(control,
            (visualTurn == TASK3_TURN_LEFT) ?
            "TASK3 NEXT LEFT\r\n" : "TASK3 NEXT RIGHT\r\n");
    }

    if (control->state == TASK3_WAIT_INFO) {
        return;
    }

    if (control->state == TASK3_WAIT_LOAD) {
        if (statusPressed) {
            control->rampSamples = 0U;
            control->intersectionArmed = true;
            control->io.setLineTrackingSpeed(0);
            control->io.setLineTrackingEnabled(true);
            control->io.resetLineTracking();
            control->state = TASK3_FOLLOW;
            Task3Control_log(control, "TASK3 OUTBOUND STARTED\r\n");
        }
        return;
    }

    switch (control->state) {
        case TASK3_FOLLOW:
            activeCount = control->io.readActiveChannelCount();

            if (activeCount < TASK3_INTERSECTION_THRESHOLD) {
                control->intersectionArmed = true;
            }

            if (control->intersectionArmed &&
                Task3Control_detectIntersection(
                    control, activeCount) &&
                (control->pendingTurn != TASK3_TURN_NONE) &&
                (control->completedTurns < TASK3_REQUIRED_TURNS)) {
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(false);
                control->io.resetLineTracking();
                control->advanceSamples = 0U;
                control->state = TASK3_ADVANCE;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_FORWARD,
                    TASK3_OUTBOUND_CRUISE_RPM, 100U);
                Task3Control_log(control,
                    "TASK3 INTERSECTION ADVANCE 220ms\r\n");
                break;
            }

            Task3Control_followLine(control);

            if (control->completedTurns >= TASK3_REQUIRED_TURNS) {
                if (activeCount > 0U) {
                    control->lineSeenAfterFinalTurn = true;
                    control->blankSamples = 0U;
                } else if (control->lineSeenAfterFinalTurn) {
                    if (control->blankSamples < 255U) {
                        control->blankSamples++;
                    }
                    if (control->blankSamples >=
                        TASK3_BLANK_CONFIRM_SAMPLES) {
                        Task3Control_beginPidBrake(control,
                            TASK3_BRAKE_END);
                        Task3Control_log(control,
                            "TASK3 END BRAKING\r\n");
                    }
                }
            }
            break;

        case TASK3_RETURN_FOLLOW:
        {
            Task3Control_Turn returnTurn;

            activeMask = control->io.readActiveChannelMask();
            activeCount =
                Task3Control_countActiveChannels(activeMask);
            returnTurn =
                Task3Control_detectReturnJunction(activeMask);

            /*
             * After a turn, first leave the old junction before allowing
             * another half-array detection. This prevents counting one
             * physical T junction twice.
             */
            if (!Task3Control_returnJunctionPresent(activeMask)) {
                control->intersectionArmed = true;
            }

            if ((control->completedReturnTurns <
                    TASK3_REQUIRED_TURNS) &&
                control->intersectionArmed &&
                (returnTurn != TASK3_TURN_NONE)) {
                control->pendingTurn = returnTurn;
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(false);
                control->io.resetLineTracking();
                control->advanceSamples = 0U;
                control->state = TASK3_ADVANCE;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_FORWARD,
                    TASK3_RETURN_CRUISE_RPM, 100U);
                Task3Control_log(control,
                    (returnTurn == TASK3_TURN_LEFT) ?
                    "TASK3 RETURN T LEFT, ADVANCE\r\n" :
                    "TASK3 RETURN T RIGHT, ADVANCE\r\n");
                break;
            }

            Task3Control_followLine(control);

            if (control->completedReturnTurns >=
                TASK3_REQUIRED_TURNS) {
                if (activeCount > 0U) {
                    control->lineSeenAfterFinalTurn = true;
                    control->blankSamples = 0U;
                } else if (control->lineSeenAfterFinalTurn) {
                    if (control->blankSamples < 255U) {
                        control->blankSamples++;
                    }
                    if (control->blankSamples >=
                        TASK3_BLANK_CONFIRM_SAMPLES) {
                        Task3Control_beginPidBrake(control,
                            TASK3_BRAKE_END);
                        Task3Control_log(control,
                            "TASK3 RETURN START BRAKING\r\n");
                    }
                }
            }
            break;
        }

        case TASK3_ADVANCE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_FORWARD,
                control->returning ? TASK3_RETURN_CRUISE_RPM :
                    TASK3_OUTBOUND_CRUISE_RPM,
                100U);
            control->advanceSamples++;
            if (control->advanceSamples >=
                TASK3_ADVANCE_SAMPLES) {
                Task3Control_beginPidBrake(control,
                    TASK3_BRAKE_TURN);
                Task3Control_log(control,
                    "TASK3 TURN PID BRAKING\r\n");
            }
            break;

        case TASK3_BRAKE_TURN:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK3_MIN_BRAKE_SAMPLES) &&
                Task3Control_carStopped(control)) {
                bool turnLeft =
                    (control->pendingTurn == TASK3_TURN_LEFT);

                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        turnLeft, TASK3_TURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->pendingTurn = TASK3_TURN_NONE;
                    control->uTurn = false;
                    control->state = TASK3_TURNING;
                    Task3Control_log(control, turnLeft ?
                        "TASK3 LEFT TURN STARTED\r\n" :
                        "TASK3 RIGHT TURN STARTED\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
                    control->io.setRedLed(true);
                    Task3Control_log(control,
                        "TASK3 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >=
                TASK3_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(true);
                Task3Control_log(control,
                    "TASK3 TURN BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_TURNING:
            if (turnResult == ANGLE_TURN_RESULT_COMPLETED) {
                control->rampSamples = 0U;
                control->intersectionConfirmSamples = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterFinalTurn = false;
                /*
                 * The sensors may still cover the same cross immediately
                 * after turning. Re-arm only after fewer than six channels
                 * are active so this cross cannot count twice.
                 */
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(true);
                control->io.resetLineTracking();

                if (control->uTurn) {
                    control->uTurn = false;
                    control->completedReturnTurns = 0U;
                    control->state = TASK3_RETURN_FOLLOW;
                    Task3Control_log(control,
                        "TASK3 UTURN DONE, RETURN FOLLOWING\r\n");
                } else if (control->returning) {
                    control->completedReturnTurns++;
                    control->state = TASK3_RETURN_FOLLOW;
                    Task3Control_log(control,
                        (control->completedReturnTurns >=
                            TASK3_REQUIRED_TURNS) ?
                        "TASK3 RETURN TURN2 DONE, FOLLOW START\r\n" :
                        "TASK3 RETURN TURN1 DONE\r\n");
                } else {
                    control->completedTurns++;
                    control->state = TASK3_FOLLOW;
                    Task3Control_log(control,
                        (control->completedTurns >=
                            TASK3_REQUIRED_TURNS) ?
                        "TASK3 TURN2 DONE, FOLLOW TO END\r\n" :
                        "TASK3 TURN1 DONE, WAIT NEXT COMMAND\r\n");
                }
            } else if ((turnResult ==
                    ANGLE_TURN_RESULT_TIMEOUT) ||
                (turnResult == ANGLE_TURN_RESULT_FAULT)) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(true);
                Task3Control_log(control, "TASK3 TURN FAULT\r\n");
            }
            break;

        case TASK3_BRAKE_END:
            control->brakeSamples++;
            if ((control->brakeSamples >=
                    TASK3_MIN_BRAKE_SAMPLES) &&
                Task3Control_carStopped(control)) {
                control->reverseSamples = 0U;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_BACKWARD, TASK3_REVERSE_RPM, 100U);
                control->state = TASK3_REVERSE;
                Task3Control_log(control,
                    "TASK3 REVERSE 75RPM 500ms\r\n");
            } else if (control->brakeSamples >=
                TASK3_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                control->io.setRedLed(true);
                Task3Control_log(control,
                    "TASK3 END BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_REVERSE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_BACKWARD, TASK3_REVERSE_RPM, 100U);
            control->reverseSamples++;
            if (control->reverseSamples >= TASK3_REVERSE_SAMPLES) {
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
                    if (control->returning) {
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
                            "TASK3 ARRIVED, WAIT UNLOAD\r\n");
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
                control->io.setRedLed(true);
                Task3Control_log(control,
                    "TASK3 FINAL BRAKE TIMEOUT\r\n");
            }
            break;

        case TASK3_WAIT_UNLOAD:
            if (statusReleased) {
                control->io.setRedLed(false);
                control->io.setGreenLed(false);
                control->pendingTurn = TASK3_TURN_NONE;
                control->completedReturnTurns = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterFinalTurn = false;
                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        true, TASK3_RETURN_TURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->returning = true;
                    control->uTurn = true;
                    control->state = TASK3_TURNING;
                    Task3Control_log(control,
                        "TASK3 RETURN UTURN STARTED 180deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
                    control->io.setRedLed(true);
                    Task3Control_log(control,
                        "TASK3 RETURN UTURN START FAILED\r\n");
                }
            }
            break;

        case TASK3_DONE:
        case TASK3_FAULT:
        default:
            break;
    }
}
