#include "oled.h"

#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDRESS       (0x3CU)
#define OLED_I2C_TIMEOUT_LOOPS (200000U)

static bool OLED_waitIdle(void)
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

static bool OLED_writeByte(uint8_t control, uint8_t value)
{
    uint8_t packet[2] = {control, value};
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    if (!OLED_waitIdle()) {
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

static bool OLED_command(uint8_t command)
{
    return OLED_writeByte(0x00U, command);
}

static bool OLED_data(uint8_t data)
{
    return OLED_writeByte(0x40U, data);
}

static bool OLED_setPosition(uint8_t column, uint8_t page)
{
    return OLED_command((uint8_t) (0xB0U + page)) &&
           OLED_command((uint8_t) (column & 0x0FU)) &&
           OLED_command((uint8_t) (0x10U | (column >> 4)));
}

static bool OLED_clear(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        if (!OLED_setPosition(0U, page)) {
            return false;
        }
        for (column = 0U; column < 128U; column++) {
            if (!OLED_data(0x00U)) {
                return false;
            }
        }
    }
    return true;
}

static const uint8_t *OLED_getGlyph(char character)
{
    static const uint8_t space[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t A[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t C[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
    static const uint8_t D[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t E[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t K[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
    static const uint8_t L[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t N[5] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
    static const uint8_t S[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t T[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
    static const uint8_t one[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t two[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
    static const uint8_t three[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
    static const uint8_t four[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
    static const uint8_t five[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};

    switch (character) {
        case '1': return one;
        case '2': return two;
        case '3': return three;
        case '4': return four;
        case '5': return five;
        case 'A': return A;
        case 'C': return C;
        case 'D': return D;
        case 'E': return E;
        case 'K': return K;
        case 'L': return L;
        case 'N': return N;
        case 'S': return S;
        case 'T': return T;
        default: return space;
    }
}

static bool OLED_writeString(uint8_t column, uint8_t page, const char *text)
{
    uint8_t glyphColumn;

    if (!OLED_setPosition(column, page)) {
        return false;
    }

    while (*text != '\0') {
        const uint8_t *glyph = OLED_getGlyph(*text);
        for (glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
            if (!OLED_data(glyph[glyphColumn])) {
                return false;
            }
        }
        if (!OLED_data(0x00U)) {
            return false;
        }
        text++;
    }
    return true;
}

bool OLED_Init(void)
{
    static const uint8_t initCommands[] = {
        0xAEU, 0x20U, 0x02U, 0x40U, 0x81U, 0xCFU, 0xA1U, 0xC8U,
        0xA6U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0xD5U, 0x80U, 0xD9U,
        0xF1U, 0xDAU, 0x12U, 0xDBU, 0x40U, 0x8DU, 0x14U, 0xA4U,
        0xAFU
    };
    uint8_t index;

    for (index = 0U; index < sizeof(initCommands); index++) {
        if (!OLED_command(initCommands[index])) {
            return false;
        }
    }
    return OLED_clear();
}

bool OLED_ShowTask(uint8_t taskNumber)
{
    char taskText[] = "TASK 1";

    if ((taskNumber < 1U) || (taskNumber > 3U)) {
        return false;
    }

    taskText[5] = (char) ('0' + taskNumber);
    return OLED_clear() && OLED_writeString(46U, 3U, taskText);
}

bool OLED_ShowTask1Endpoint(uint8_t endpointNumber)
{
    char endpointText[] = "TASK1 END1";

    if ((endpointNumber < 1U) || (endpointNumber > 2U)) {
        return false;
    }

    endpointText[9] = (char) ('0' + endpointNumber);
    return OLED_clear() && OLED_writeString(34U, 3U, endpointText);
}

bool OLED_ShowCalibration(uint8_t secondsRemaining)
{
    char calibrationText[] = "CAL 5S";

    if (secondsRemaining > 5U) {
        secondsRemaining = 5U;
    }

    calibrationText[4] = (char) ('0' + secondsRemaining);
    return OLED_clear() && OLED_writeString(46U, 3U, calibrationText);
}
