/*
 * Based on vendor MSPM0 Grayscale_Read (资料/八路灰度模块/.../Grayscale_Read):
 *   select AD[2:0] -> delay_us(50) -> read OUT
 * Hardened for this board: same-port AD0/AD1 update, OUT pull-up+hysteresis,
 * and 3-sample majority after the official settle time.
 */
#include "grayscale_sensor.h"
#include "ti_msp_dl_config.h"

#ifndef CPUCLK_FREQ
#define CPUCLK_FREQ (32000000U)
#endif

/* Official example uses 50 us after each channel select. */
#define GRAYSCALE_SETTLE_US (50U)
#define GRAYSCALE_SAMPLE_GAP_US (5U)

#define SENSOR_AD0_PORT GPIO_GRAYSCALE_AD0_PORT
#define SENSOR_AD0_PIN GPIO_GRAYSCALE_AD0_PIN
#define SENSOR_AD1_PORT GPIO_GRAYSCALE_AD1_PORT
#define SENSOR_AD1_PIN GPIO_GRAYSCALE_AD1_PIN
#define SENSOR_AD2_PORT GPIO_GRAYSCALE_AD2_PORT
#define SENSOR_AD2_PIN GPIO_GRAYSCALE_AD2_PIN
#define SENSOR_OUT_PORT GPIO_GRAYSCALE_OUT_PORT
#define SENSOR_OUT_PIN GPIO_GRAYSCALE_OUT_PIN

static void grayscale_delay_us(uint32_t microseconds)
{
    /* Same 32 MHz assumption as vendor delay_us / empty.syscfg. */
    delay_cycles(microseconds * (CPUCLK_FREQ / 1000000U));
}

/*
 * Truth table (vendor manual / CD4051):
 *   channel AD2 AD1 AD0 -> X(channel+1)
 * Official write order: AD0, AD1, AD2.
 * AD0/AD1 share GPIOB here, so update them in one clear/set pair to avoid
 * long intermediate mux addresses between bit writes.
 */
static void grayscale_select_channel(uint8_t channel)
{
    const uint8_t ad0 = (uint8_t) ((channel >> 0) & 0x01U);
    const uint8_t ad1 = (uint8_t) ((channel >> 1) & 0x01U);
    const uint8_t ad2 = (uint8_t) ((channel >> 2) & 0x01U);

    /* Prefer one-shot AD0/AD1 update when they share a GPIO port (current wiring). */
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
    /* Same as vendor: !!(DL_GPIO_readPins(port, pin)) */
    return (uint8_t) (!!(DL_GPIO_readPins(SENSOR_OUT_PORT, SENSOR_OUT_PIN)));
}

/* Extra stability under motor PWM / encoder EMI; not in bare vendor demo. */
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
     * Re-apply OUT as input with pull-up + hysteresis.
     * Vendor uses floating input; on a running chassis, OUT is quieter with
     * a weak pull-up and Schmitt trigger.
     */
    DL_GPIO_initDigitalInputFeatures(GPIO_GRAYSCALE_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(SENSOR_AD0_PORT, SENSOR_AD0_PIN | SENSOR_AD1_PIN);
    DL_GPIO_clearPins(SENSOR_AD2_PORT, SENSOR_AD2_PIN);
}

void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t channel;

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        grayscale_select_channel(channel);
        grayscale_delay_us(GRAYSCALE_SETTLE_US);
        values[channel] = grayscale_read_out_majority();
    }
}

uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
        return 0U;
    }

    grayscale_select_channel(channel);
    grayscale_delay_us(GRAYSCALE_SETTLE_US);
    return grayscale_read_out_majority();
}
