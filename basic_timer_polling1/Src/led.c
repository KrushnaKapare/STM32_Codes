/*
 * led.c
 *
 *  Created on: Sep 13, 2025
 *      Author: sunbeam
 */
#include"led.h"

void led_init(uint8_t pin)
{
	//	12 - 24 & 25	//	pin * 2 & pin * 2 + 1
	//	13 - 26 & 27
	//1. Clock Enable
	RCC->AHB1ENR |= BV(GPIOD_CLK_EN);
	//2. Mode - output
	LED_GPIO->MODER |= BV(pin * 2);
	LED_GPIO->MODER &= ~BV(pin * 2 + 1);
	//3. Output Type - push pull
	LED_GPIO->OTYPER &= ~BV(pin);
	//4. Output Speed - low
	LED_GPIO->OSPEEDR &= ~(BV(pin * 2) | BV(pin * 2 + 1));
	//5. pull up/down - no
	LED_GPIO->PUPDR &= ~(BV(pin * 2) | BV(pin * 2 + 1));
}

void led_on(uint8_t pin)
{
	LED_GPIO->ODR |= BV(pin);
}

void led_off(uint8_t pin)
{
	LED_GPIO->ODR &= ~BV(pin);
}

void led_toggle(uint8_t pin)
{
	LED_GPIO->ODR ^= BV(pin);
}












