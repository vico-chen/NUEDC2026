/*
 * 8-ch digital grayscale (CD4051), based on vendor MSPM0 Grayscale_Read.
 * Pins come from SysConfig GPIO_GRAYSCALE: AD0/AD1/AD2/OUT.
 */
#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include <stdint.h>

#define GRAYSCALE_SENSOR_CHANNELS (8U)

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_ReadAll(uint8_t values[GRAYSCALE_SENSOR_CHANNELS]);
uint8_t Grayscale_Sensor_ReadChannel(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H_ */
