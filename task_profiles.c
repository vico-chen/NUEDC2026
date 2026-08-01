#include "task_profiles.h"

/*
 * ======================== 环形巡线任务参数 ========================
 *
 * accelerationSamples、decelerationSamples 的单位是 10 ms。
 * finishAdvanceMm 是扫到终点十字路口后继续前进的距离。
 * wheelDiameterMm 必须按实际轮胎有效直径校准。
 */
 
 // 只有2有停车偏差，所以跑快点然后不用减速
const LapTask_Profile gLapTask2Profile = {
    .cruiseRpm = 150,
    .straightRpm = 280,
    .lostLineRecoveryRpmOffset = 160,
    .accelerationSamples = 30U,
    .straightAccelerationSamples = 80U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 1U,
    .finishAdvanceMm = 100U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 10,
    .decelerationSamples = 30U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

const LapTask_Profile gLapTask5Profile = {
    .cruiseRpm = 140,
    .straightRpm = 140,
    .lostLineRecoveryRpmOffset = 0,
    .accelerationSamples = 600U,
    .straightAccelerationSamples = 0U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 1U,
    .finishAdvanceMm = 50U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 200U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

const LapTask_Profile gLapTask6Profile = {
    .cruiseRpm = 140,
    .straightRpm = 140,
    .lostLineRecoveryRpmOffset = 0,
    .accelerationSamples = 600U,
    .straightAccelerationSamples = 0U,
    .intersectionActiveThreshold = 4U,
    .intersectionConfirmSamples = 1U,
    .finishAdvanceMm = 50U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 200U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

/*
 * ======================== Task4 定距直行参数 ========================
 *
 */
const StraightTask_Profile gStraightTask4Profile = {
    .targetDistanceMm = 1950U,
    .wheelDiameterMm = 65U,
    .cruiseRpm = 180,
    .accelerationSamples = 300U,
    .decelerationDistanceMm = 800U,
    .decelerationSamples = 300U,
    .decelerationEndRpm = 5,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};
