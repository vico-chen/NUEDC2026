#include "task3_control.h"

/* 所有 samples 参数均以 10 ms 为一个计数周期。 */
#define TASK3_OUTBOUND_CRUISE_RPM         (150)
#define TASK3_RETURN_CRUISE_RPM           (110)
#define TASK3_RAMP_SAMPLES                (20U) /* 0.20 s */
#define TASK3_INTERSECTION_THRESHOLD      (6U)
#define TASK3_INTERSECTION_CONFIRM        (2U)
#define TASK3_ADVANCE_SAMPLES             (16U) /* 0.22 s */
#define TASK3_MIN_BRAKE_SAMPLES           (15U)
#define TASK3_BRAKE_TIMEOUT_SAMPLES       (80U)
#define TASK3_TURN_DEGREES                (85.0f)
#define TASK3_TURN_RPM                    (100)
#define TASK3_REQUIRED_TURNS              (2U)
#define TASK3_PRESET_FIRST_TURN_CROSS      (3U)
#define TASK3_RETURN_TURN_DEGREES         (180.0f)
#define TASK3_LEFT_SENSOR_MASK            (0x0FU) /* X1..X4 */
#define TASK3_RIGHT_SENSOR_MASK           (0xF0U) /* X5..X8 */
#define TASK3_BLANK_CONFIRM_SAMPLES       (10U)
#define TASK3_REVERSE_RPM                 (75)
#define TASK3_REVERSE_SAMPLES             (50U) /* 0.50 s */
#define TASK3_FINAL_SETTLE_SAMPLES        (5U)

static void Task3Control_log(Task3Control *control, const char *text)
{
    if (control->io.log != 0) {
        control->io.log(text);
    }
}

/* 四个编码器速度均进入死区后，才判定车辆已经停稳。 */
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

/* 去程和返程采用不同巡航速度，并各自从 0 线性加速。 */
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

/* 通过有效灰度通道数量和连续帧数确认十字路口。 */
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

/* 返程时根据左四路或右四路全有效判断 T 形路口方向。 */
static Task2Control_Turn Task3Control_detectReturnJunction(
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
        return TASK2_TURN_NONE;
    }
    return leftDetected ? TASK2_TURN_LEFT : TASK2_TURN_RIGHT;
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

/*
 * Endpoint 预设的两次去程转向：
 *   END1：左、左    END2：左、右
 *   END3：右、左    END4：右、右
 * AUTO 返回 NONE，由 OpenMV 提供原方案中的转向指令。
 */
static Task2Control_Turn Task3Control_getPresetTurn(
    TaskManager_Task3Endpoint endpoint, uint8_t turnIndex)
{
    if ((endpoint == TASK_MANAGER_TASK3_ENDPOINT_AUTO) ||
        (turnIndex >= TASK3_REQUIRED_TURNS)) {
        return TASK2_TURN_NONE;
    }

    if (turnIndex == 0U) {
        return ((endpoint == TASK_MANAGER_TASK3_ENDPOINT_1) ||
                (endpoint == TASK_MANAGER_TASK3_ENDPOINT_2)) ?
            TASK2_TURN_LEFT : TASK2_TURN_RIGHT;
    }

    return ((endpoint == TASK_MANAGER_TASK3_ENDPOINT_1) ||
            (endpoint == TASK_MANAGER_TASK3_ENDPOINT_3)) ?
        TASK2_TURN_LEFT : TASK2_TURN_RIGHT;
}

/* 关闭巡线，使用电机零速闭环制动，并切换到指定状态。 */
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
    control->completedReturnTurns = 0U;
    control->outboundIntersectionsSeen = 0U;
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
    control->visionNumberReady = false;
    control->selectedEndpoint =
        TASK_MANAGER_TASK3_ENDPOINT_AUTO;
    control->io.setLineTrackingEnabled(false);
    control->io.resetLineTracking();
    AngleTurnControl_cancel(control->io.angleTurn);
    CarControl_stop(control->io.car);
}

void Task3Control_init(Task3Control *control,
    const Task1Control_Config *io)
{
    control->io = *io;
    Task3Control_reset(control);
}

void Task3Control_update(Task3Control *control, TaskManager_Task task,
    TaskManager_Task3Endpoint endpoint, bool statusPressed,
    bool statusReleased, bool numberReceived,
    Task2Control_Turn visualTurn, bool mpuReady,
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

    if (numberReceived) {
        control->visionNumberReady = true;
    }

    /*
     * 预设 endpoint 不依赖 OpenMV 数字，选择后即可等待装载；
     * AUTO 则保持原方案，必须先收到视觉数字。
     */
    if (control->state == TASK3_WAIT_INFO) {
        if (endpoint != TASK_MANAGER_TASK3_ENDPOINT_AUTO) {
            control->state = TASK3_WAIT_LOAD;
            Task3Control_log(control,
                "TASK3 ENDPOINT READY, WAIT LOAD\r\n");
        } else if (control->visionNumberReady) {
            control->state = TASK3_WAIT_LOAD;
            Task3Control_log(control,
                "TASK3 VISION NUMBER READY\r\n");
        }
    } else if ((control->state == TASK3_WAIT_LOAD) &&
               (endpoint == TASK_MANAGER_TASK3_ENDPOINT_AUTO) &&
               !control->visionNumberReady) {
        /* 从预设端点切回 AUTO 时，重新等待 OpenMV 数字。 */
        control->state = TASK3_WAIT_INFO;
    }

    if ((visualTurn != TASK2_TURN_NONE) &&
        (control->selectedEndpoint ==
            TASK_MANAGER_TASK3_ENDPOINT_AUTO) &&
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
            /* 启动后锁定本次端点，运行中按钮变化不影响当前路线。 */
            control->selectedEndpoint = endpoint;
            control->pendingTurn =
                Task3Control_getPresetTurn(endpoint, 0U);
            control->outboundIntersectionsSeen = 0U;
            control->rampSamples = 0U;
            control->intersectionArmed = true;
            control->io.setLineTrackingSpeed(0);
            control->io.setLineTrackingEnabled(true);
            control->io.resetLineTracking();
            control->state = TASK3_FOLLOW;
            if (endpoint == TASK_MANAGER_TASK3_ENDPOINT_AUTO) {
                Task3Control_log(control,
                    "TASK3 AUTO OUTBOUND STARTED\r\n");
            } else {
                Task3Control_log(control,
                    "TASK3 ENDPOINT OUTBOUND STARTED\r\n");
            }
        }
        return;
    }

    switch (control->state) {
        /* 去程巡线：预设路线从第三个十字路口开始执行两次转向。 */
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
                /*
                 * 预设 endpoint 的第一次转向从第三个十字路口开始。
                 * 前两个十字路口只计数并保持直行；离开当前路口后
                 * intersectionArmed 才会重新置位，避免同一路口重复计数。
                 */
                if ((control->selectedEndpoint !=
                        TASK_MANAGER_TASK3_ENDPOINT_AUTO) &&
                    (control->completedTurns == 0U)) {
                    if (control->outboundIntersectionsSeen < 255U) {
                        control->outboundIntersectionsSeen++;
                    }
                    if (control->outboundIntersectionsSeen <
                        TASK3_PRESET_FIRST_TURN_CROSS) {
                        control->intersectionArmed = false;
                        Task3Control_followLine(control);
                        Task3Control_log(control,
                            "TASK3 PRESET CROSS PASSED STRAIGHT\r\n");
                        break;
                    }
                }

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

        /* 返程巡线：按倒序和反方向还原去程的两次转向。 */
        case TASK3_RETURN_FOLLOW:
        {
            Task2Control_Turn returnTurn;

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
                (returnTurn != TASK2_TURN_NONE)) {
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
                    (returnTurn == TASK2_TURN_LEFT) ?
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

        /* 直行进入路口中心，使旋转中心尽量落在交叉区域。 */
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

        /* 转弯前等待四轮停稳，然后启动定角转向。 */
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
                    control->uTurn = false;
                    control->state = TASK3_TURNING;
                    Task3Control_log(control, turnLeft ?
                        "TASK3 LEFT TURN STARTED 85deg\r\n" :
                        "TASK3 RIGHT TURN STARTED\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
                    Task3Control_log(control,
                        "TASK3 TURN START FAILED\r\n");
                }
            } else if (control->brakeSamples >=
                TASK3_BRAKE_TIMEOUT_SAMPLES) {
                CarControl_emergencyStop(control->io.car);
                control->state = TASK3_FAULT;
                Task3Control_log(control,
                    "TASK3 TURN BRAKE TIMEOUT\r\n");
            }
            break;

        /* 处理普通 85 度转向和卸载后的 180 度掉头结果。 */
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
                    if (control->selectedEndpoint !=
                        TASK_MANAGER_TASK3_ENDPOINT_AUTO) {
                        control->pendingTurn =
                            Task3Control_getPresetTurn(
                                control->selectedEndpoint,
                                control->completedTurns);
                    }
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
                Task3Control_log(control, "TASK3 TURN FAULT\r\n");
            }
            break;

        /* 确认终点停车，随后进入定时倒车调整。 */
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
                Task3Control_log(control,
                    "TASK3 END BRAKE TIMEOUT\r\n");
            }
            break;

        /* 固定速度倒车 0.5 秒。 */
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

        /* 倒车结束后再次制动，连续稳定多帧才算真正停稳。 */
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
                        Task3Control_log(control,
                            "TASK3 RETURN DONE\r\n");
                    } else {
                        control->state = TASK3_WAIT_UNLOAD;
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
                Task3Control_log(control,
                    "TASK3 FINAL BRAKE TIMEOUT\r\n");
            }
            break;

        /* 释放卸载按钮后右转 180 度，开始按原路返程。 */
        case TASK3_WAIT_UNLOAD:
            if (statusReleased) {
                control->pendingTurn = TASK2_TURN_NONE;
                control->completedReturnTurns = 0U;
                control->blankSamples = 0U;
                control->lineSeenAfterFinalTurn = false;
                if (mpuReady &&
                    AngleTurnControl_start(control->io.angleTurn,
                        false, TASK3_RETURN_TURN_DEGREES,
                        TASK3_TURN_RPM)) {
                    control->returning = true;
                    control->uTurn = true;
                    control->state = TASK3_TURNING;
                    Task3Control_log(control,
                        "TASK3 RETURN UTURN STARTED 180deg\r\n");
                } else {
                    CarControl_emergencyStop(control->io.car);
                    control->state = TASK3_FAULT;
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
