/*
 * led.c
 *
 *  Created on: Sep 13, 2025
 *      Author: krushna
 */
#include "led.h"
void led_init(uint8_t pin)
{

	//CLOCK ENABLE 0,1,2,3 OF BV IS SET TO 1 BECAUSE OF OR OPERATION
	RCC->AHB1ENR |= BV(GPIOD_CLK_EN);

	//1. ENABLE THE MODE REGISTER
	LED_GPIO->MODER |= BV(pin * 2);
	LED_GPIO->MODER &= ~ BV(pin * 2 +1);

	//2. OUTPUT TYPE
	LED_GPIO->OTYPER &= ~BV(pin);

	LED_GPIO->OSPEEDR &= ~(BV(pin *2) | BV(pin * 2 +1));

	//4 PULL UP PULL DOWN
	LED_GPIO->PUPDR &= ~(BV(pin *2) | BV(pin * 2 +1));

}
void led_on(uint8_t pin)
{
	LED_GPIO->ODR |= BV(pin);
}
void led_off(uint8_t pin)
{
	LED_GPIO->ODR &= ~(BV(pin));
}
void led_toggle(uint8_t pin)
{
	LED_GPIO->ODR ^=  BV(pin);

}
