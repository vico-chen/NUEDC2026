/*
 * 八路灰度传感器采集驱动。
 *
 * 基本时序沿用厂商 MSPM0 示例 Grayscale_Read：
 *   通过 AD[2:0] 选择通道 -> 等待 50 us -> 读取 OUT。
 *
 * 针对小车运行时的电机 PWM 和编码器干扰，本实现额外做了三项增强：
 *   1. AD0、AD1 位于同一 GPIO 端口时一次性更新，避免多路复用器短暂选错通道；
 *   2. OUT 输入启用弱上拉和施密特迟滞；
 *   3. 每个通道连续读取三次并采用多数表决。
 */
#include "grayscale_sensor.h"
#include "board_pins.h"

#ifndef CPUCLK_FREQ
#define CPUCLK_FREQ (32000000U)
#endif

/* 每次切换多路复用器通道后，按官方示例等待 50 us 使信号稳定。 */
#define GRAYSCALE_SETTLE_US (50U)
#define GRAYSCALE_SAMPLE_GAP_US (5U)

#define SENSOR_AD0_PORT BOARD_GRAYSCALE_AD0_PORT
#define SENSOR_AD0_PIN BOARD_GRAYSCALE_AD0_PIN
#define SENSOR_AD1_PORT BOARD_GRAYSCALE_AD1_PORT
#define SENSOR_AD1_PIN BOARD_GRAYSCALE_AD1_PIN
#define SENSOR_AD2_PORT BOARD_GRAYSCALE_AD2_PORT
#define SENSOR_AD2_PIN BOARD_GRAYSCALE_AD2_PIN
#define SENSOR_OUT_PORT BOARD_GRAYSCALE_OUT_PORT
#define SENSOR_OUT_PIN BOARD_GRAYSCALE_OUT_PIN

static void grayscale_delay_us(uint32_t microseconds)
{
    /* 系统主频为 32 MHz，因此 1 us 对应 32 个 CPU 周期。 */
    delay_cycles(microseconds * (CPUCLK_FREQ / 1000000U));
}

/*
 * CD4051 通道选择真值表：
 *   channel 的二进制位 AD2 AD1 AD0 对应 X(channel + 1)。
 *
 * 厂商示例按 AD0、AD1、AD2 的顺序写入。当前硬件的 AD0 和 AD1 共用
 * 一个 GPIO 端口，因此优先通过一次清零、一次置位完成更新，避免逐位写入
 * 期间出现持续时间较长的中间地址。
 */
static void grayscale_select_channel(uint8_t channel)
{
    const uint8_t ad0 = (uint8_t) ((channel >> 0) & 0x01U);
    const uint8_t ad1 = (uint8_t) ((channel >> 1) & 0x01U);
    const uint8_t ad2 = (uint8_t) ((channel >> 2) & 0x01U);

    /* 同端口时合并更新；若以后换到不同端口，自动退化为逐引脚写入。 */
    if (SENSOR_AD0_PORT == SENSOR_AD1_PORT) {
        const uint32_t ad01_pins = SENSOR_AD0_PIN | SENSOR_AD1_PIN;
        uint32_t ad01_high = 0U;

        if (ad0 != 0U) {
            ad01_high |= SENSOR_AD0_PIN;
        }
        if (ad1 != 0U) {
            ad01_high |= SENSOR_AD1_PIN;
        }

        DL_GPIO_clearPins(SENSOR_AD0_PORT, ad01_pins);
        if (ad01_high != 0U) {
            DL_GPIO_setPins(SENSOR_AD0_PORT, ad01_high);
        }
    } else {
        if (ad0 != 0U) {
            DL_GPIO_setPins(SENSOR_AD0_PORT, SENSOR_AD0_PIN);
        } else {
            DL_GPIO_clearPins(SENSOR_AD0_PORT, SENSOR_AD0_PIN);
        }
        if (ad1 != 0U) {
            DL_GPIO_setPins(SENSOR_AD1_PORT, SENSOR_AD1_PIN);
        } else {
            DL_GPIO_clearPins(SENSOR_AD1_PORT, SENSOR_AD1_PIN);
        }
    }

    if (ad2 != 0U) {
        DL_GPIO_setPins(SENSOR_AD2_PORT, SENSOR_AD2_PIN);
    } else {
        DL_GPIO_clearPins(SENSOR_AD2_PORT, SENSOR_AD2_PIN);
    }
}

static uint8_t grayscale_read_out_raw(void)
{
    /* 双重取反把 GPIO 位掩码统一转换成 0 或 1。 */
    return (uint8_t) (!!(DL_GPIO_readPins(SENSOR_OUT_PORT, SENSOR_OUT_PIN)));
}

/* 三次采样多数表决，提高电机 PWM 和编码器电磁干扰下的稳定性。 */
static uint8_t grayscale_read_out_majority(void)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;

    a = grayscale_read_out_raw();
    grayscale_delay_us(GRAYSCALE_SAMPLE_GAP_US);
    b = grayscale_read_out_raw();
    grayscale_delay_us(GRAYSCALE_SAMPLE_GAP_US);
    c = grayscale_read_out_raw();

    return (uint8_t) (((uint8_t) (a + b + c) >= 2U) ? 1U : 0U);
}

void Grayscale_Sensor_Init(void)
{
    /*
     * 重新把 OUT 配置为带弱上拉和施密特迟滞的数字输入。
     * 先显式关闭输出驱动：即使以后在 SysConfig 中误把 OUT 配成输出，
     * 初始化过程也不会让 MCU 与传感器同时驱动该信号线。
     */
    DL_GPIO_disableOutput(SENSOR_OUT_PORT, SENSOR_OUT_PIN);
    DL_GPIO_initDigitalInputFeatures(BOARD_GRAYSCALE_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 上电默认选择 0 号通道，给后续第一次读取一个确定的初始状态。 */
    DL_GPIO_clearPins(SENSOR_AD0_PORT, SENSOR_AD0_PIN);
    DL_GPIO_clearPins(SENSOR_AD1_PORT, SENSOR_AD1_PIN);
    DL_GPIO_clearPins(SENSOR_AD2_PORT, SENSOR_AD2_PIN);
}

void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t channel;

    /* values[0]～values[7] 与多路复用器通道 0～7 一一对应。 */
    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        grayscale_select_channel(channel);
        grayscale_delay_us(GRAYSCALE_SETTLE_US);
        values[channel] = grayscale_read_out_majority();
    }
}

uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel)
{
    /* 越界通道不访问硬件，直接返回低电平。 */
    if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
        return 0U;
    }

    grayscale_select_channel(channel);
    grayscale_delay_us(GRAYSCALE_SETTLE_US);
    return grayscale_read_out_majority();
}
