#include "task_profiles.h"

/* 同事最终版本的任务参数；后续赛道实测只需集中修改本文件。 */
const LapTask_Profile gLapTask2Profile = {
    .cruiseRpm = 150,
    .straightRpm = 280,
    .accelerationSamples = 80U,
    .straightAccelerationSamples = 50U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 2U,
    .finishAdvanceMm = 100U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 10,
    .decelerationSamples = 30U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

const LapTask_Profile gLapTask5Profile = {
    .cruiseRpm = 130,
    .straightRpm = 130,
    .accelerationSamples = 600U,
    .straightAccelerationSamples = 0U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 1U,
    .finishAdvanceMm = 100U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 100U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

const LapTask_Profile gLapTask6Profile = {
    .cruiseRpm = 130,
    .straightRpm = 130,
    .accelerationSamples = 600U,
    .straightAccelerationSamples = 0U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 1U,
    .finishAdvanceMm = 100U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 100U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

const StraightTask_Profile gStraightTask4Profile = {
    .targetDistanceMm = 2000U,
    .wheelDiameterMm = 65U,
    .cruiseRpm = 130,
    .accelerationSamples = 300U,
    .decelerationDistanceMm = 150U,
    .decelerationEndRpm = 5,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};
