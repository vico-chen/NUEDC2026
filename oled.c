#include "oled.h"

#include "stopwatch.h"
#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDRESS       (0x3CU)
#define OLED_I2C_TIMEOUT_LOOPS (200000U)

/* 等待 OLED 所在的 I2C 控制器空闲。 */
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

/* 发送一个命令或数据字节；control 用来区分两种类型。 */
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
    /* SSD1306 按页寻址，每页高度为 8 像素。 */
    return OLED_command((uint8_t) (0xB0U + page)) &&
           OLED_command((uint8_t) (column & 0x0FU)) &&
           OLED_command((uint8_t) (0x10U | (column >> 4)));
}

static bool OLED_clear(void)
{
    /* 清空 8 页 × 128 列的全部显示显存。 */
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
    /* 5×7 字模只保存当前界面实际使用的数字和大写字母。 */
    static const uint8_t space[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t A[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t C[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
    static const uint8_t D[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t E[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t K[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
    static const uint8_t L[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t N[5] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
    static const uint8_t O[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    static const uint8_t R[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
    static const uint8_t S[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t T[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
    static const uint8_t U[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
    static const uint8_t one[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t two[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
    static const uint8_t three[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
    static const uint8_t four[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
    static const uint8_t five[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};
    static const uint8_t zero[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
    static const uint8_t six[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U};
    static const uint8_t seven[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U};
    static const uint8_t eight[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t nine[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU};

    switch (character) {
        case '0': return zero;
        case '1': return one;
        case '2': return two;
        case '3': return three;
        case '4': return four;
        case '5': return five;
        case '6': return six;
        case '7': return seven;
        case '8': return eight;
        case '9': return nine;
        case 'A': return A;
        case 'C': return C;
        case 'D': return D;
        case 'E': return E;
        case 'K': return K;
        case 'L': return L;
        case 'N': return N;
        case 'O': return O;
        case 'R': return R;
        case 'S': return S;
        case 'T': return T;
        case 'U': return U;
        default: return space;
    }
}

static bool OLED_writeString(uint8_t column, uint8_t page, const char *text)
{
    /* 每字写 5 列字模，再写 1 列空白作为字符间距。 */
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

/*
 * 以 3 倍比例绘制 5×7 字模：
 * 每个原始像素扩展为 3×3 像素，最终字符大小约为 15×21。
 * 21 像素高度横跨 OLED 的三个页，因此逐页输出低、中、高字节。
 */
static bool OLED_writeLargeString(
    uint8_t column, uint8_t page, const char *text)
{
    uint8_t pageOffset;

    for (pageOffset = 0U; pageOffset < 3U; pageOffset++) {
        const char *character = text;

        if (!OLED_setPosition(column, (uint8_t) (page + pageOffset))) {
            return false;
        }

        while (*character != '\0') {
            const uint8_t *glyph = OLED_getGlyph(*character);
            uint8_t glyphColumn;

            for (glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
                uint32_t enlargedColumn = 0U;
                uint8_t row;
                uint8_t repeat;

                /* 将原字模的一列从 7 像素纵向放大到 21 像素。 */
                for (row = 0U; row < 7U; row++) {
                    if ((glyph[glyphColumn] & (1U << row)) != 0U) {
                        enlargedColumn |= (uint32_t) 0x07U << (row * 3U);
                    }
                }

                /* 每一列横向重复三次，完成 3 倍宽度放大。 */
                for (repeat = 0U; repeat < 3U; repeat++) {
                    if (!OLED_data((uint8_t)
                            (enlargedColumn >> (pageOffset * 8U)))) {
                        return false;
                    }
                }
            }

            /* 字符之间保留三列空白。 */
            for (glyphColumn = 0U; glyphColumn < 3U; glyphColumn++) {
                if (!OLED_data(0x00U)) {
                    return false;
                }
            }
            character++;
        }
    }
    return true;
}

bool OLED_Init(void)
{
    /* 配置寻址、扫描方向、对比度和电荷泵，最后打开显示。 */
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

    if ((taskNumber < 1U) || (taskNumber > 6U)) {
        return false;
    }

    taskText[5] = (char) ('0' + taskNumber);
    return OLED_clear() && OLED_writeString(46U, 3U, taskText);
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

bool OLED_ShowStopwatch(uint64_t stopwatchMilliseconds)
{
    char stopwatchText[6];
    uint32_t seconds;
    uint32_t firstDigit;
    uint8_t textLength;
    uint8_t column;

    /* 最多显示 9999S，超过后保持在 9999S，避免内容超出屏幕。 */
    seconds = (uint32_t) (stopwatchMilliseconds / 1000U);
    if (seconds > 9999U) {
        seconds = 9999U;
    }

    /* 不使用 printf，直接从高位到低位生成无前导零的整数文本。 */
    if (seconds >= 1000U) {
        firstDigit = 1000U;
    } else if (seconds >= 100U) {
        firstDigit = 100U;
    } else if (seconds >= 10U) {
        firstDigit = 10U;
    } else {
        firstDigit = 1U;
    }

    textLength = 0U;
    while (firstDigit > 0U) {
        stopwatchText[textLength] =
            (char) ('0' + ((seconds / firstDigit) % 10U));
        textLength++;
        firstDigit /= 10U;
    }
    stopwatchText[textLength++] = 'S';
    stopwatchText[textLength] = '\0';

    /* 每个大字符占 18 列，把类似“12S”的内容水平居中。 */
    column = (uint8_t) ((128U - ((uint16_t) textLength * 18U)) / 2U);
    return OLED_clear() &&
        OLED_writeLargeString(column, 2U, stopwatchText);
}
