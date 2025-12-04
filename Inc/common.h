#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f401xc.h"
#include "changeLabButton.h"
#include "multiFuncButton.h"
#include "lab1LedStrip.h"
#include "lab2PwmAdc.h"
#include "uart.h"
#include "lab3UartAdcDma.h"

#define HSI_SOURCE 0
#define HSE_SOURCE 1
#define PLL_SOURCE 2

#define NUM_OF_SAMPLES 128

#define NUM_OF_LABS 3

void cpuFreqInit(uint8_t freqSource);
void sysTickInit(void);
void delayMs(uint32_t msTime);
uint8_t digitExtractor(uint16_t value, uint8_t digit);

#endif /* COMMON_H_ */
