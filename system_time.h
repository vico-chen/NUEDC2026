#ifndef SYSTEM_TIME_H_
#define SYSTEM_TIME_H_

#include <stdint.h>

/* 由 10 ms 定时器中断维护的统一系统时基。 */
void SystemTime_tick10ms(void);
uint32_t SystemTime_getMs(void);

#endif /* SYSTEM_TIME_H_ */
