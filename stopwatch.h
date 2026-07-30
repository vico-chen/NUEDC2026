#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <stdbool.h>
#include <stdint.h>

void Stopwatch_start(void);
void Stopwatch_stop(void);
void Stopwatch_reset(void);
uint32_t Stopwatch_getElapsedMs(void);
bool Stopwatch_isRunning(void);

#endif /* STOPWATCH_H_ */
