#ifndef ANGLE_TURN_CONTROL_H_
#define ANGLE_TURN_CONTROL_H_

#include "car_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ANGLE_TURN_RESULT_NONE,
    ANGLE_TURN_RESULT_COMPLETED,
    ANGLE_TURN_RESULT_TIMEOUT
} AngleTurnControl_Result;

typedef struct {
    CarControl *car;
    int8_t leftTurnYawSign;
    float kpRpmPerDegree;
    float kdRpmPerDps;
    int16_t minimumRpm;
    int16_t defaultMaximumRpm;
    float angleToleranceDegrees;
    /*
     * After the controller enters the stop/hold window, it keeps braking
     * until |error| exceeds this larger hysteresis band. Prevents hunting
     * around the tolerance edge.
     */
    float holdReleaseDegrees;
    /*
     * Below this remaining error, do not force minimumRpm. Tiny residual
     * error should brake/coast instead of kicking the chassis again.
     */
    float minRpmEngageDegrees;
    float stoppedRateToleranceDps;
    uint16_t settleSamples;
    uint16_t timeoutSamples;
} AngleTurnControl_Config;

typedef struct {
    AngleTurnControl_Config config;
    float targetYawDegrees;
    int16_t maximumRpm;
    uint16_t settledCount;
    uint16_t elapsedSamples;
    bool active;
    bool holding;
} AngleTurnControl;

void AngleTurnControl_init(
    AngleTurnControl *control, const AngleTurnControl_Config *config);
bool AngleTurnControl_start(AngleTurnControl *control, bool turnLeft,
    float relativeAngleDegrees, int16_t maximumRpm);
AngleTurnControl_Result AngleTurnControl_update(AngleTurnControl *control);
void AngleTurnControl_cancel(AngleTurnControl *control);
bool AngleTurnControl_isActive(const AngleTurnControl *control);
float AngleTurnControl_getErrorDegrees(const AngleTurnControl *control);

#endif /* ANGLE_TURN_CONTROL_H_ */
