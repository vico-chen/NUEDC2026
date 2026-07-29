#include "stopwatch.h"

/* 本次计时的起点与停止时刻，单位均为毫秒。 */
static uint64_t startSystick = 0;
static uint64_t stopSystick = 0;

/* true 表示秒表正在运行，false 表示保持停止时的读数。 */
static bool isEnabled = false;

/* 从当前系统时基开始一次新的计时。 */
void Stopwatch_start(void)
{
    startSystick = systick_ms;
    isEnabled = true;
}

/* 返回已经经过的毫秒数；停止后读数保持不变。 */
uint64_t Stopwatch_getCurrentMs(void)
{
    if (!isEnabled)
        return stopSystick - startSystick;
    return systick_ms - startSystick;
}

/* 保存停止时刻并冻结本次秒表读数。 */
void Stopwatch_stop(void)
{
    stopSystick = systick_ms;
    isEnabled = false;
}

/* 查询秒表当前是否正在运行。 */
bool Stopwatch_isEnabled(void)
{
    return isEnabled;
}
