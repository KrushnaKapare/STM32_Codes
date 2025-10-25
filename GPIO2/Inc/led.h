/*
 * led.h
 *
 *  Created on: Sep 13, 2025
 *      Author: krushna
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f407xx.h"
#define BV(n) (1 << (n))

void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);


#endif /* LED_H_ */
