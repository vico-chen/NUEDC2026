#include "system_time.h"

static volatile uint32_t gSystemTimeMs;

void SystemTime_tick10ms(void)
{
    gSystemTimeMs += 10U;
}

uint32_t SystemTime_getMs(void)
{
    return gSystemTimeMs;
}
