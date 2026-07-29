#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 由主程序的 10 ms 定时中断累加，秒表模块只读取该系统时基。 */
extern volatile uint64_t systick_ms;

/* 开始计时、读取当前经过时间、停止计时以及查询运行状态。 */
void Stopwatch_start(void);
uint64_t Stopwatch_getCurrentMs(void);
void Stopwatch_stop(void);
bool Stopwatch_isEnabled(void);

#endif
