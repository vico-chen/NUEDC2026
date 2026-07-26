#include "task3_control.h"

#define TASK3_CRUISE_RPM                  (110)
#define TASK3_RAMP_SAMPLES                (20U) /* 0.20 s */
#define TASK3_INTERSECTION_THRESHOLD      (6U)
#define TASK3_INTERSECTION_CONFIRM        (2U)
#define TASK3_ADVANCE_SAMPLES             (22U) /* 0.22 s */
#define TASK3_MIN_BRAKE_SAMPLES           (15U)
#define TASK3_BRAKE_TIMEOUT_SAMPLES       (80U)
#define TASK3_TURN_DEGREES                (85.0f)
#define TASK3_TURN_RPM                    (100)
#define TASK3_REQUIRED_TURNS              (2U)
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
    if (control->rampSamples < TASK3_RAMP_SAMPLES) {
        control->rampSamples++;
    }
    control->io.setLineTrackingSpeed((int16_t) (((int32_t)
        TASK3_CRUISE_RPM * control->rampSamples) /
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
    control->pendingTurn = TASK2_TURN_NONE;
    control->completedTurns = 0U;
    control->rampSamples = 0U;
    control->intersectionConfirmSamples = 0U;
    control->advanceSamples = 0U;
    control->blankSamples = 0U;
    control->reverseSamples = 0U;
    control->settledSamples = 0U;
    control->brakeSamples = 0U;
    control->lineSeenAfterSecondTurn = false;
    control->intersectionArmed = false;
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
    bool statusPressed, bool numberReceived,
    Task2Control_Turn visualTurn, bool mpuReady,
    AngleTurnControl_Result turnResult)
{
    uint8_t activeCount;

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

    if ((visualTurn != TASK2_TURN_NONE) &&
        (control->state == TASK3_FOLLOW) &&
        (control->completedTurns < TASK3_REQUIRED_TURNS) &&
        control->intersectionArmed &&
        (control->pendingTurn == TASK2_TURN_NONE)) {
        control->pendingTurn = visualTurn;
        Task3Control_log(control,
            (visualTurn == TASK2_TURN_LEFT) ?
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
                (control->pendingTurn != TASK2_TURN_NONE) &&
                (control->completedTurns < TASK3_REQUIRED_TURNS)) {
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(false);
                control->io.resetLineTracking();
                control->advanceSamples = 0U;
                control->state = TASK3_ADVANCE;
                CarControl_setMotion(control->io.car,
                    CAR_CONTROL_FORWARD, TASK3_CRUISE_RPM, 100U);
                Task3Control_log(control,
                    "TASK3 INTERSECTION ADVANCE 220ms\r\n");
                break;
            }

            Task3Control_followLine(control);

            if (control->completedTurns >= TASK3_REQUIRED_TURNS) {
                if (activeCount > 0U) {
                    control->lineSeenAfterSecondTurn = true;
                    control->blankSamples = 0U;
                } else if (control->lineSeenAfterSecondTurn) {
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

        case TASK3_ADVANCE:
            CarControl_setMotion(control->io.car,
                CAR_CONTROL_FORWARD, TASK3_CRUISE_RPM, 100U);
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
                    (control->pendingTurn == TASK2_TURN_LEFT);

                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        turnLeft, TASK3_TURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->pendingTurn = TASK2_TURN_NONE;
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
                control->completedTurns++;
                control->rampSamples = 0U;
                control->intersectionConfirmSamples = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterSecondTurn = false;
                /*
                 * The sensors may still cover the same cross immediately
                 * after turning. Re-arm only after fewer than six channels
                 * are active so this cross cannot count twice.
                 */
                control->intersectionArmed = false;
                control->io.setLineTrackingEnabled(true);
                control->io.resetLineTracking();
                control->state = TASK3_FOLLOW;
                Task3Control_log(control,
                    (control->completedTurns >=
                        TASK3_REQUIRED_TURNS) ?
                    "TASK3 TURN2 DONE, FOLLOW TO END\r\n" :
                    "TASK3 TURN1 DONE, WAIT NEXT COMMAND\r\n");
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
                    control->state = TASK3_WAIT_UNLOAD;
                    control->io.setRedLed(true);
                    control->io.setGreenLed(false);
                    Task3Control_log(control,
                        "TASK3 ARRIVED, WAIT UNLOAD\r\n");
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
        case TASK3_FAULT:
        default:
            break;
    }
}
