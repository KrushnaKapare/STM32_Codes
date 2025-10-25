/*
 * clock.h
 *
 *  Created on: Oct 1, 2025
 *      Author: krushna
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "stm32f4xx.h"
#include "stm32f407xx.h"

#define HSECLK 8000000UL
#define CCLK  168000000UL
#define PCLK1 42000000UL
#define PCLK2 84000000UL

#define PLL_P 2
#define PLL_M 8
#define PLL_Q 7
#define PLL_N 336

#define PRE_H 1
#define PRE_P1 4
#define PRE_P2 2

void Clock_Setup(void);

#endif /* CLOCK_H_ */
