/*
 * LP-MSPM0G3507 平台的 MPU6050 相对 Z 轴角度驱动。
 *
 * I2C 读写遵循 TI 重复起始条件的标准传输时序。单次事务数据量很小，
 * 因此采用带超时保护的轮询方式，不额外占用中断资源。
 *
 * 初始化时对 Z 轴陀螺仪做静止零偏标定；运行时以固定 10 ms 周期读取角速度，
 * 去除零偏后进行积分，得到小车自复位时刻起的相对转角。
 */

#include "mpu6050_angle.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

#define MPU6050_I2C_ADDRESS             (0x68U)
#define MPU6050_REG_SMPLRT_DIV          (0x19U)
#define MPU6050_REG_CONFIG              (0x1AU)
#define MPU6050_REG_GYRO_CONFIG         (0x1BU)
#define MPU6050_REG_GYRO_ZOUT_H         (0x47U)
#define MPU6050_REG_PWR_MGMT_1          (0x6BU)
#define MPU6050_REG_WHO_AM_I            (0x75U)

#define MPU6050_DEVICE_ID_CLASSIC       (0x68U)
#define MPU6050_DEVICE_ID_NEW           (0x70U)
#define MPU6050_GYRO_FS_500DPS          (0x08U)
#define MPU6050_GYRO_LSB_PER_DPS        (65.5f)
#define MPU6050_SAMPLE_PERIOD_SECONDS   (0.010f)
#define MPU6050_CALIBRATION_SAMPLES     (500U)
#define MPU6050_I2C_TIMEOUT_LOOPS       (200000U)
#define MPU6050_POWER_UP_DELAY_CYCLES   (3200000U) /* 32 MHz 下延时 100 ms */
#define MPU6050_SAMPLE_DELAY_CYCLES     (320000U)  /* 32 MHz 下延时 10 ms */

/* 角度模块内部状态：零偏原始值、积分角度、当前角速度和芯片 ID。 */
static float gGyroZBiasRaw;
static float gZAngleDegrees;
static float gZRateDps;
static uint8_t gDeviceId;

static bool MPU6050_waitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;

    /* 等待控制器空闲；超时可避免 I2C 总线异常时程序永久卡死。 */
    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
                DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static bool MPU6050_waitTransferDone(uint32_t doneInterrupt)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;

    /* 同时监视“传输完成”和“控制器错误”两个状态。 */
    while (timeout > 0U) {
        if ((DL_I2C_getRawInterruptStatus(
                 I2C_MPU6050_INST, doneInterrupt) & doneInterrupt) != 0U) {
            return true;
        }
        if ((DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
        }
        timeout--;
    }
    return false;
}

static bool MPU6050_writeRegister(uint8_t registerAddress, uint8_t value)
{
    /* MPU6050 单寄存器写入格式：[寄存器地址，数据]。 */
    uint8_t packet[2] = {registerAddress, value};

    if (!MPU6050_waitIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_clearInterruptStatus(
        I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (DL_I2C_fillControllerTXFIFO(
            I2C_MPU6050_INST, packet, sizeof(packet)) != sizeof(packet)) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(packet));
    /* 规避 MSPM0 I2C_ERR_13：启动传输后等待超过 3 个 I2C 时钟。 */
    delay_cycles(100U);

    return MPU6050_waitTransferDone(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
}

static bool MPU6050_readRegisters(
    uint8_t registerAddress, uint8_t *data, uint8_t length)
{
    uint8_t received = 0U;
    uint32_t timeout;

    if ((length == 0U) || !MPU6050_waitIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_flushControllerRXFIFO(I2C_MPU6050_INST);
    DL_I2C_clearInterruptStatus(
        I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    DL_I2C_transmitControllerData(I2C_MPU6050_INST, registerAddress);

    /*
     * 第一阶段只发送寄存器地址，不发送 STOP；第二阶段以重复 START
     * 切换到接收方向，连续读取 length 字节。
     */
    DL_I2C_startControllerTransferAdvanced(I2C_MPU6050_INST,
        MPU6050_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(100U);

    if (!MPU6050_waitTransferDone(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
        return false;
    }

    DL_I2C_clearInterruptStatus(
        I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_startControllerTransferAdvanced(I2C_MPU6050_INST,
        MPU6050_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, length,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(100U);

    /* 持续搬运 RX FIFO 数据，并在总线错误或超时时复位本次传输。 */
    timeout = MPU6050_I2C_TIMEOUT_LOOPS;
    while ((received < length) && (timeout > 0U)) {
        while ((received < length) &&
               !DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
            data[received] =
                DL_I2C_receiveControllerData(I2C_MPU6050_INST);
            received++;
        }
        if ((DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            break;
        }
        timeout--;
    }

    if (received != length) {
        DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
        return false;
    }
    return MPU6050_waitTransferDone(DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
}

static bool MPU6050_readGyroZRaw(int16_t *gyroZRaw)
{
    uint8_t data[2];

    if (!MPU6050_readRegisters(MPU6050_REG_GYRO_ZOUT_H, data, 2U)) {
        return false;
    }

    /* 传感器输出为高字节在前的 16 位有符号二进制补码。 */
    *gyroZRaw = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    return true;
}

bool MPU6050_Angle_initWithProgress(
    MPU6050_CalibrationProgressCallback progressCallback)
{
    int64_t gyroZSum = 0;
    uint16_t sample;
    int16_t gyroZRaw;

    gGyroZBiasRaw = 0.0f;
    gZAngleDegrees = 0.0f;
    gZRateDps = 0.0f;
    gDeviceId = 0U;

    if (progressCallback != 0) {
        progressCallback(5U);
    }

    /* 等待传感器上电稳定，并先通过 WHO_AM_I 判断器件是否在线。 */
    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);
    if (!MPU6050_readRegisters(MPU6050_REG_WHO_AM_I, &gDeviceId, 1U) ||
        ((gDeviceId != MPU6050_DEVICE_ID_CLASSIC) &&
         (gDeviceId != MPU6050_DEVICE_ID_NEW))) {
        return false;
    }

    /* 软件复位，清除传感器之前可能遗留的配置状态。 */
    if (!MPU6050_writeRegister(MPU6050_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);

    /*
     * 时钟源选 X 轴陀螺仪 PLL；
     * 采样率 = 1 kHz / (9 + 1) = 100 Hz；
     * DLPF_CFG = 3，陀螺仪带宽约 44 Hz；
     * 量程设为 ±500 °/s，对应 65.5 LSB/(°/s)。
     */
    if (!MPU6050_writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01U) ||
        !MPU6050_writeRegister(MPU6050_REG_SMPLRT_DIV, 9U) ||
        !MPU6050_writeRegister(MPU6050_REG_CONFIG, 3U) ||
        !MPU6050_writeRegister(
            MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500DPS)) {
        return false;
    }

    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);

    /*
     * 连续采集 500 个样本求平均值作为 Z 轴零偏。
     * 该过程约 5 秒，期间必须让小车完全静止，否则会产生固定角速度误差。
     */
    for (sample = 0U; sample < MPU6050_CALIBRATION_SAMPLES; sample++) {
        if ((progressCallback != 0) && (sample > 0U) &&
            ((sample % 100U) == 0U)) {
            progressCallback((uint8_t) (
                (MPU6050_CALIBRATION_SAMPLES - sample) / 100U));
        }
        if (!MPU6050_readGyroZRaw(&gyroZRaw)) {
            return false;
        }
        gyroZSum += gyroZRaw;
        delay_cycles(MPU6050_SAMPLE_DELAY_CYCLES);
    }

    gGyroZBiasRaw =
        (float) gyroZSum / (float) MPU6050_CALIBRATION_SAMPLES;
    if (progressCallback != 0) {
        progressCallback(0U);
    }
    return true;
}

bool MPU6050_Angle_init(void)
{
    return MPU6050_Angle_initWithProgress(0);
}

bool MPU6050_Angle_update(void)
{
    int16_t gyroZRaw;

    if (!MPU6050_readGyroZRaw(&gyroZRaw)) {
        return false;
    }

    /* 原始值减去静止零偏后换算成 °/s，再按 10 ms 周期积分。 */
    gZRateDps =
        ((float) gyroZRaw - gGyroZBiasRaw) / MPU6050_GYRO_LSB_PER_DPS;
    gZAngleDegrees += gZRateDps * MPU6050_SAMPLE_PERIOD_SECONDS;
    return true;
}

void MPU6050_Angle_reset(void)
{
    /* 只清除相对角度，保留上电标定得到的零偏。 */
    gZAngleDegrees = 0.0f;
}

float MPU6050_Angle_getZDegrees(void)
{
    return gZAngleDegrees;
}

float MPU6050_Angle_getZRateDps(void)
{
    return gZRateDps;
}

unsigned char MPU6050_Angle_getDeviceId(void)
{
    return gDeviceId;
}
