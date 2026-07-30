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

/*
 * ======================== Task4 定距直行参数 ========================
 *
 * 目标距离为 1500 mm。最后 300 mm 根据剩余编码器距离连续减速，
 * 降至 50 RPM 后，在达到目标距离时进入制动和停稳确认。
 */
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
