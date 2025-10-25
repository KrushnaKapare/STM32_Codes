/*
 * timer.h
 *
 *  Created on: Sep 29, 2025
 *      Author: krushna
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xx.h"

#define FPCLK 16000000UL
#define PR 16000

void TimerInit(void);
void TimerDelayMs(uint32_t ms);

#endif /* TIMER_H_ */
