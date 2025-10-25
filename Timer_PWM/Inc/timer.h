/*
 * timer.h
 *
 *  Created on: Sep 30, 2025
 *      Author: krushna
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xx.h"


#define TIM_PCLK  16000000
#define TIM_PR 16

void TimerPwmInit(void);


#endif /* TIMER_H_ */
