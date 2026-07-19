/*
 * MPU6050 relative Z-axis angle driver for LP-MSPM0G3507.
 *
 * I2C access follows TI's
 * i2c_controller_rw_repeated_start_fifo_interrupts transfer sequence, using
 * polling because each transaction is only one or six bytes. Register setup
 * follows the supplied 51/Arduino examples. The stationary bias calibration
 * and turn-angle integration are adapted for the car application.
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
#define MPU6050_POWER_UP_DELAY_CYCLES   (3200000U) /* 100 ms at 32 MHz */
#define MPU6050_SAMPLE_DELAY_CYCLES     (320000U)  /* 10 ms at 32 MHz */

static float gGyroZBiasRaw;
static float gZAngleDegrees;
static float gZRateDps;
static uint8_t gDeviceId;

static bool MPU6050_waitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;

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
    delay_cycles(100U); /* MSPM0 I2C_ERR_13 workaround: >3 I2C clocks. */

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

    *gyroZRaw = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    return true;
}

bool MPU6050_Angle_init(void)
{
    int64_t gyroZSum = 0;
    uint16_t sample;
    int16_t gyroZRaw;

    gGyroZBiasRaw = 0.0f;
    gZAngleDegrees = 0.0f;
    gZRateDps = 0.0f;
    gDeviceId = 0U;

    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);
    if (!MPU6050_readRegisters(MPU6050_REG_WHO_AM_I, &gDeviceId, 1U) ||
        ((gDeviceId != MPU6050_DEVICE_ID_CLASSIC) &&
         (gDeviceId != MPU6050_DEVICE_ID_NEW))) {
        return false;
    }

    if (!MPU6050_writeRegister(MPU6050_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);

    /*
     * Clock = X gyro PLL, sample rate = 1 kHz/(9+1) = 100 Hz,
     * DLPF_CFG=3 (~44 Hz gyro bandwidth), gyro range = +/-500 dps.
     */
    if (!MPU6050_writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01U) ||
        !MPU6050_writeRegister(MPU6050_REG_SMPLRT_DIV, 9U) ||
        !MPU6050_writeRegister(MPU6050_REG_CONFIG, 3U) ||
        !MPU6050_writeRegister(
            MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500DPS)) {
        return false;
    }

    delay_cycles(MPU6050_POWER_UP_DELAY_CYCLES);

    /* Keep the car completely still during this approximately 5 s step. */
    for (sample = 0U; sample < MPU6050_CALIBRATION_SAMPLES; sample++) {
        if (!MPU6050_readGyroZRaw(&gyroZRaw)) {
            return false;
        }
        gyroZSum += gyroZRaw;
        delay_cycles(MPU6050_SAMPLE_DELAY_CYCLES);
    }

    gGyroZBiasRaw =
        (float) gyroZSum / (float) MPU6050_CALIBRATION_SAMPLES;
    return true;
}

bool MPU6050_Angle_update(void)
{
    int16_t gyroZRaw;

    if (!MPU6050_readGyroZRaw(&gyroZRaw)) {
        return false;
    }

    gZRateDps =
        ((float) gyroZRaw - gGyroZBiasRaw) / MPU6050_GYRO_LSB_PER_DPS;
    gZAngleDegrees += gZRateDps * MPU6050_SAMPLE_PERIOD_SECONDS;
    return true;
}

void MPU6050_Angle_reset(void)
{
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
