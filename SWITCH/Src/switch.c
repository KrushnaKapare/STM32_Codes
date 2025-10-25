/*
 * switch.c
 *
 *  Created on: Sep 15, 2025
 *      Author: krushna
 */


#include"switch.h"
#define BV(n) (1<<(n))
void switch_init(uint8_t pin)
{
	RCC->AHB1ENR |= BV(SWITCH_CLK_EN);
	SWITCH_GPIO->MODER &= ~(BV(pin *2) | BV(pin *2 +1));
	SWITCH_GPIO->PUPDR &= ~(BV(pin *2) | BV(pin *2 +1));

}

uint8_t  switch_status(void)
{
	return SWITCH_GPIO->IDR & BV(SWITCH_PIN) ? 1 : 0;

}
