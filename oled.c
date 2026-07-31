#include "oled.h"

#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDRESS       (0x3CU)
#define OLED_I2C_TIMEOUT_LOOPS (200000U)
#define OLED_PAGE_COUNT        (8U)
#define OLED_COLUMN_COUNT      (128U)

/*
 * OLED 只在主循环中访问。使用静态缓冲区避免 512 字节系统栈同时承受
 * 页面渲染缓冲和 I2C 发送缓冲，电机中断抢占时也更安全。
 */
static uint8_t gOledPacket[OLED_COLUMN_COUNT + 1U];
static uint8_t gOledRenderBuffer[OLED_COLUMN_COUNT];

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

static bool OLED_writePacket(
    uint8_t control, const uint8_t *values, uint16_t length)
{
    uint16_t totalLength = (uint16_t) (length + 1U);
    uint16_t sentLength;
    uint16_t index;
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    if ((values == 0) || (length == 0U) ||
        (length > OLED_COLUMN_COUNT)) {
        return false;
    }
    gOledPacket[0] = control;
    for (index = 0U; index < length; index++) {
        gOledPacket[index + 1U] = values[index];
    }

    if (!OLED_waitIdle()) {
        return false;
    }
    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(
        I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    sentLength = DL_I2C_fillControllerTXFIFO(
        I2C_OLED_INST, gOledPacket, totalLength);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, totalLength);
    /* MSPM0 I2C_ERR_13 规避：启动传输后保留硬件建立时间。 */
    delay_cycles(100U);

    while (timeout > 0U) {
        if (sentLength < totalLength) {
            sentLength += DL_I2C_fillControllerTXFIFO(
                I2C_OLED_INST, &gOledPacket[sentLength],
                (uint16_t) (totalLength - sentLength));
        }
        if ((DL_I2C_getRawInterruptStatus(I2C_OLED_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) &
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            return sentLength == totalLength;
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
    return OLED_writePacket(0x00U, &command, 1U);
}

static bool OLED_writeDataBlock(
    const uint8_t *data, uint16_t length)
{
    return OLED_writePacket(0x40U, data, length);
}

static bool OLED_setPosition(uint8_t column, uint8_t page)
{
    return OLED_command((uint8_t) (0xB0U + page)) &&
           OLED_command((uint8_t) (column & 0x0FU)) &&
           OLED_command((uint8_t) (0x10U | (column >> 4)));
}

static bool OLED_clearPages(uint8_t firstPage, uint8_t pageCount)
{
    static const uint8_t zeroPage[OLED_COLUMN_COUNT] = {0U};
    uint8_t page;
    uint8_t endPage = (uint8_t) (firstPage + pageCount);

    if (endPage > OLED_PAGE_COUNT) {
        endPage = OLED_PAGE_COUNT;
    }
    for (page = firstPage; page < endPage; page++) {
        if (!OLED_setPosition(0U, page)) {
            return false;
        }
        if (!OLED_writeDataBlock(
                zeroPage, OLED_COLUMN_COUNT)) {
            return false;
        }
    }
    return true;
}

static const uint8_t *OLED_getGlyph(char character)
{
    static const uint8_t blank[5] =
        {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t digits[10][5] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
    };
    static const uint8_t letters[26][5] = {
        {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* A */
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* B */
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* C */
        {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* D */
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* E */
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* F */
        {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* G */
        {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* H */
        {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* I */
        {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* J */
        {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* K */
        {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* L */
        {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* M */
        {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* N */
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* O */
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* P */
        {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* Q */
        {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* R */
        {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* S */
        {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* T */
        {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* U */
        {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* V */
        {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}, /* W */
        {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* X */
        {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, /* Y */
        {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}  /* Z */
    };

    if ((character >= '0') && (character <= '9')) {
        return digits[(uint8_t) (character - '0')];
    }
    if ((character >= 'A') && (character <= 'Z')) {
        return letters[(uint8_t) (character - 'A')];
    }
    return blank;
}

static uint8_t OLED_textLength(const char *text)
{
    uint8_t length = 0U;

    while ((*text != '\0') && (length < 21U)) {
        length++;
        text++;
    }
    return length;
}

static bool OLED_writeString(
    uint8_t column, uint8_t page, const char *text)
{
    uint16_t outputLength = 0U;
    uint8_t glyphColumn;

    while ((*text != '\0') &&
           ((outputLength + 6U) <= OLED_COLUMN_COUNT)) {
        const uint8_t *glyph = OLED_getGlyph(*text);
        for (glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
            gOledRenderBuffer[outputLength++] = glyph[glyphColumn];
        }
        gOledRenderBuffer[outputLength++] = 0x00U;
        text++;
    }
    return (outputLength > 0U) &&
           OLED_setPosition(column, page) &&
           OLED_writeDataBlock(gOledRenderBuffer, outputLength);
}

static bool OLED_writeCentered(uint8_t page, const char *text)
{
    uint16_t width = (uint16_t) OLED_textLength(text) * 6U;
    uint8_t column = (width < OLED_COLUMN_COUNT) ?
        (uint8_t) ((OLED_COLUMN_COUNT - width) / 2U) : 0U;

    return OLED_writeString(column, page, text);
}

static bool OLED_writeLargeString(
    uint8_t column, uint8_t page, const char *text)
{
    uint8_t pageOffset;

    for (pageOffset = 0U; pageOffset < 3U; pageOffset++) {
        uint16_t outputLength = 0U;
        const char *character = text;

        while ((*character != '\0') &&
               ((outputLength + 18U) <= OLED_COLUMN_COUNT)) {
            const uint8_t *glyph = OLED_getGlyph(*character);
            uint8_t glyphColumn;

            for (glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
                uint32_t enlargedColumn = 0U;
                uint8_t row;
                uint8_t repeat;

                for (row = 0U; row < 7U; row++) {
                    if ((glyph[glyphColumn] & (1U << row)) != 0U) {
                        enlargedColumn |=
                            (uint32_t) 0x07U << (row * 3U);
                    }
                }
                for (repeat = 0U; repeat < 3U; repeat++) {
                    gOledRenderBuffer[outputLength++] = (uint8_t)
                        (enlargedColumn >> (pageOffset * 8U));
                }
            }
            for (glyphColumn = 0U; glyphColumn < 3U; glyphColumn++) {
                gOledRenderBuffer[outputLength++] = 0x00U;
            }
            character++;
        }
        if ((outputLength == 0U) ||
            !OLED_setPosition(
                column, (uint8_t) (page + pageOffset)) ||
            !OLED_writeDataBlock(
                gOledRenderBuffer, outputLength)) {
            return false;
        }
    }
    return true;
}

static void OLED_makeTaskText(uint8_t taskNumber, char text[7])
{
    text[0] = 'T';
    text[1] = 'A';
    text[2] = 'S';
    text[3] = 'K';
    text[4] = ' ';
    text[5] = (char) ('0' + taskNumber);
    text[6] = '\0';
}

static uint8_t OLED_makeSecondsText(
    uint32_t elapsedMs, char text[6])
{
    uint32_t seconds = elapsedMs / 1000U;
    uint32_t divisor;
    uint8_t length = 0U;

    if (seconds > 9999U) {
        seconds = 9999U;
    }
    if (seconds >= 1000U) {
        divisor = 1000U;
    } else if (seconds >= 100U) {
        divisor = 100U;
    } else if (seconds >= 10U) {
        divisor = 10U;
    } else {
        divisor = 1U;
    }
    while (divisor > 0U) {
        text[length++] =
            (char) ('0' + ((seconds / divisor) % 10U));
        divisor /= 10U;
    }
    text[length++] = 'S';
    text[length] = '\0';
    return length;
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
    return OLED_clearPages(0U, OLED_PAGE_COUNT);
}

bool OLED_ShowMessage(const char *text)
{
    if (text == 0) {
        return false;
    }
    return OLED_clearPages(0U, OLED_PAGE_COUNT) &&
           OLED_writeCentered(3U, text);
}

bool OLED_ShowCalibration(uint8_t secondsRemaining)
{
    char text[7] = "CAL 5S";

    if (secondsRemaining > 9U) {
        secondsRemaining = 9U;
    }
    text[4] = (char) ('0' + secondsRemaining);
    return OLED_ShowMessage(text);
}

bool OLED_ShowTask(uint8_t taskNumber)
{
    char taskText[7];

    if ((taskNumber < 1U) || (taskNumber > 6U)) {
        return false;
    }
    OLED_makeTaskText(taskNumber, taskText);
    return OLED_clearPages(0U, OLED_PAGE_COUNT) &&
           OLED_writeCentered(3U, taskText);
}

bool OLED_ShowStopwatch(uint8_t taskNumber, uint32_t elapsedMs)
{
    char taskText[11] = "TASK 1 RUN";
    char secondsText[6];
    uint8_t textLength;
    uint8_t column;

    if ((taskNumber < 1U) || (taskNumber > 6U)) {
        return false;
    }
    taskText[5] = (char) ('0' + taskNumber);
    textLength = OLED_makeSecondsText(elapsedMs, secondsText);
    column = (uint8_t) (
        (OLED_COLUMN_COUNT - ((uint16_t) textLength * 18U)) / 2U);

    /*
     * 标题只占第 0 页，秒表只清理并重画第 2～4 页。
     * 相比每次全屏清空，显著减少任务运行时的 I2C 阻塞。
     */
    return OLED_clearPages(0U, 1U) &&
           OLED_writeCentered(0U, taskText) &&
           OLED_clearPages(2U, 3U) &&
           OLED_writeLargeString(column, 2U, secondsText);
}

bool OLED_ShowTaskResult(uint8_t taskNumber,
    OLED_TaskResult result, uint32_t elapsedMs)
{
    char taskText[7];
    char secondsText[6];
    const char *resultText;
    uint8_t textLength;
    uint8_t column;

    if ((taskNumber < 1U) || (taskNumber > 6U)) {
        return false;
    }
    if (result == OLED_TASK_RESULT_DONE) {
        resultText = "DONE";
    } else if (result == OLED_TASK_RESULT_FAULT) {
        resultText = "FAULT";
    } else {
        resultText = "CANCEL";
    }

    OLED_makeTaskText(taskNumber, taskText);
    textLength = OLED_makeSecondsText(elapsedMs, secondsText);
    column = (uint8_t) (
        (OLED_COLUMN_COUNT - ((uint16_t) textLength * 18U)) / 2U);

    return OLED_clearPages(0U, OLED_PAGE_COUNT) &&
           OLED_writeCentered(0U, taskText) &&
           OLED_writeCentered(2U, resultText) &&
           OLED_writeLargeString(column, 4U, secondsText);
}
