/*
 * switch.h
 *
 *  Created on: Sep 15, 2025
 *      Author: krushna
 */

#ifndef SWITCH_H_
#define SWITCH_H_

#include"stm32f407xx.h"


#define SWITCH_CLK_EN 0
#define SWITCH_GPIO GPIOA
#define SWITCH_PIN 0

void switch_init(uint8_t pin);
uint8_t switch_status(void);


#endif /* SWITCH_H_ */
