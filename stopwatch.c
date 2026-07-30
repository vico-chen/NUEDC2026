#include "stopwatch.h"
#include "system_time.h"

static uint32_t gStartMs;
static uint32_t gStoppedElapsedMs;
static bool gRunning;

void Stopwatch_start(void)
{
    gStartMs = SystemTime_getMs();
    gStoppedElapsedMs = 0U;
    gRunning = true;
}

void Stopwatch_stop(void)
{
    if (gRunning) {
        gStoppedElapsedMs = SystemTime_getMs() - gStartMs;
        gRunning = false;
    }
}

void Stopwatch_reset(void)
{
    gStartMs = SystemTime_getMs();
    gStoppedElapsedMs = 0U;
    gRunning = false;
}

uint32_t Stopwatch_getElapsedMs(void)
{
    return gRunning ? (SystemTime_getMs() - gStartMs)
                    : gStoppedElapsedMs;
}

bool Stopwatch_isRunning(void)
{
    return gRunning;
}
