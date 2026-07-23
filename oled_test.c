#include "oled_test.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

#define OLED_I2C_ADDRESS       (0x3CU) /* 7-bit SSD1306 address */
#define OLED_I2C_TIMEOUT_LOOPS (200000U)

static bool OLED_Test_waitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static bool OLED_Test_writeByte(uint8_t control, uint8_t value)
{
    uint8_t packet[2] = {control, value};
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    if (!OLED_Test_waitIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(
        I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (DL_I2C_fillControllerTXFIFO(
            I2C_OLED_INST, packet, sizeof(packet)) != sizeof(packet)) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(packet));
    delay_cycles(100U); /* MSPM0 I2C_ERR_13 workaround */

    while (timeout > 0U) {
        if ((DL_I2C_getRawInterruptStatus(I2C_OLED_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) &
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            return true;
        }
        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
        }
        timeout--;
    }
    return false;
}

static bool OLED_Test_command(uint8_t command)
{
    return OLED_Test_writeByte(0x00U, command);
}

static bool OLED_Test_data(uint8_t data)
{
    return OLED_Test_writeByte(0x40U, data);
}

static bool OLED_Test_setPosition(uint8_t column, uint8_t page)
{
    return OLED_Test_command((uint8_t) (0xB0U + page)) &&
           OLED_Test_command((uint8_t) (column & 0x0FU)) &&
           OLED_Test_command((uint8_t) (0x10U | (column >> 4)));
}

static bool OLED_Test_clear(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        if (!OLED_Test_setPosition(0U, page)) {
            return false;
        }
        for (column = 0U; column < 128U; column++) {
            if (!OLED_Test_data(0x00U)) {
                return false;
            }
        }
    }
    return true;
}

static const uint8_t *OLED_Test_glyph(char character)
{
    static const uint8_t space[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t H[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
    static const uint8_t W[5] = {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU};
    static const uint8_t d[5] = {0x38U, 0x44U, 0x44U, 0x48U, 0x7FU};
    static const uint8_t e[5] = {0x38U, 0x54U, 0x54U, 0x54U, 0x18U};
    static const uint8_t l[5] = {0x00U, 0x41U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t o[5] = {0x38U, 0x44U, 0x44U, 0x44U, 0x38U};
    static const uint8_t r[5] = {0x7CU, 0x08U, 0x04U, 0x04U, 0x08U};

    switch (character) {
        case 'H': return H;
        case 'W': return W;
        case 'd': return d;
        case 'e': return e;
        case 'l': return l;
        case 'o': return o;
        case 'r': return r;
        default:  return space;
    }
}

static bool OLED_Test_writeString(uint8_t column, uint8_t page,
    const char *text)
{
    uint8_t glyphColumn;

    if (!OLED_Test_setPosition(column, page)) {
        return false;
    }

    while (*text != '\0') {
        const uint8_t *glyph = OLED_Test_glyph(*text);
        for (glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
            if (!OLED_Test_data(glyph[glyphColumn])) {
                return false;
            }
        }
        if (!OLED_Test_data(0x00U)) {
            return false;
        }
        text++;
    }
    return true;
}

bool OLED_Test_initAndShowHelloWorld(void)
{
    static const uint8_t initCommands[] = {
        0xAEU, 0x20U, 0x02U, 0x40U, 0x81U, 0xCFU, 0xA1U, 0xC8U,
        0xA6U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0xD5U, 0x80U, 0xD9U,
        0xF1U, 0xDAU, 0x12U, 0xDBU, 0x40U, 0x8DU, 0x14U, 0xA4U,
        0xAFU
    };
    uint8_t index;

    for (index = 0U; index < sizeof(initCommands); index++) {
        if (!OLED_Test_command(initCommands[index])) {
            return false;
        }
    }

    return OLED_Test_clear() && OLED_Test_writeString(31U, 3U, "Hello World");
}
