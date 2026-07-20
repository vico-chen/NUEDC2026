#ifndef ANGLE_TURN_CONTROL_H_
#define ANGLE_TURN_CONTROL_H_

#include "car_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ANGLE_TURN_RESULT_NONE,
    ANGLE_TURN_RESULT_COMPLETED,
    ANGLE_TURN_RESULT_TIMEOUT,
    ANGLE_TURN_RESULT_FAULT
} AngleTurnControl_Result;

typedef struct {
    CarControl *car;
    int8_t leftTurnYawSign;
    int16_t defaultCruiseRpm;
    int16_t creepRpm;
    /*
     * Predicted coast distance = clamp(tolerance + gain * approachRate,
     *                                  tolerance, brakeAheadMax).
     */
    float brakeRateGain;
    float brakeAheadMaxDegrees;
    /* Acceptable final error for DONE. */
    float angleToleranceDegrees;
    float stoppedRateToleranceDps;
    uint16_t settleSamples;
    uint16_t timeoutSamples;
} AngleTurnControl_Config;

typedef struct {
    AngleTurnControl_Config config;
    float targetAbsDegrees;
    float turnDirection;
    int16_t cruiseRpm;
    uint16_t settledCount;
    uint16_t coastSamples;
    uint16_t elapsedSamples;
    bool active;
    bool coasting;
    bool creeping;
} AngleTurnControl;

void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config);
bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t cruiseRpm);
AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control);
void AngleTurnControl_cancel(AngleTurnControl *control);
bool AngleTurnControl_isActive(const AngleTurnControl *control);
float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control);

#endif /* ANGLE_TURN_CONTROL_H_ */
