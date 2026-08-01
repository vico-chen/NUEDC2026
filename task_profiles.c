#include "task_profiles.h"

/* 同事最终版本的任务参数；后续赛道实测只需集中修改本文件。 */
const LapTask_Profile gLapTask2Profile = {
    .cruiseRpm = 200,
    .straightRpm = 200,
    .accelerationSamples = 80U,
    .straightAccelerationSamples = 0U,
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
    .finishAdvanceMm = 50U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 200U,
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
    .finishAdvanceMm = 50U,
    .wheelDiameterMm = 65U,
    .decelerationEndRpm = 5,
    .decelerationSamples = 200U,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};

/*
 * Task4 的 2100 mm 是同事在 2000 mm 赛题距离上得到的实车补偿值。
 * 最后 800 mm 用 3 秒匀减速；若斜坡结束时编码器距离仍不足，
 * 保持 5 RPM 缓行到目标，避免仅按时间停车造成欠程。
 */
const StraightTask_Profile gStraightTask4Profile = {
    .targetDistanceMm = 2100U,
    .wheelDiameterMm = 65U,
    .cruiseRpm = 160,
    .accelerationSamples = 300U,
    .decelerationDistanceMm = 800U,
    .decelerationSamples = 300U,
    .decelerationEndRpm = 5,
    .brakeMinimumSamples = 15U,
    .brakeTimeoutSamples = 100U,
};
