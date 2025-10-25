/*
 * led.h
 *
 *  Created on: Apr 5, 2025
 *      Author: admin
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx.h"

#define LED_GREEN	12
#define LED_ORANGE	13
#define LED_RED		14
#define LED_BLUE	15

void LedInit(uint32_t pin);
void LedOn(uint32_t pin);
void LedOff(uint32_t pin);
void LedBlink(uint32_t pin, uint32_t ms);
void LedToggle(uint32_t pin);

#endif /* LED_H_ */
