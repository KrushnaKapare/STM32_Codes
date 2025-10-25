/*
 * led.c
 *
 *  Created on: Apr 5, 2025
 *      Author: admin
 */

#include "led.h"

void LedInit(uint32_t pin) {
	// enable gpio d clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	// change pin mode to output (01)
	GPIOD->MODER |= BV(pin*2);
	GPIOD->MODER &= ~BV(pin*2 + 1);
	// disable pullup & pulldown
	GPIOD->PUPDR &= ~(BV(pin*2) | BV(pin*2 + 1));
	// keep gpio speed low
	GPIOD->OSPEEDR &= ~(BV(pin*2) | BV(pin*2 + 1));
	// enable push-pull mode
	GPIOD->OTYPER &= ~BV(pin);
}

void LedOn(uint32_t pin) {
	GPIOD->BSRR |= BV(pin);
	//GPIOD->ODR |= BV(pin);
}

void LedOff(uint32_t pin) {
	GPIOD->BSRR |= BV(pin + 16);
	//GPIOD->ODR &= ~BV(pin);
}

void LedBlink(uint32_t pin, uint32_t ms) {
	LedOn(pin);
	DelayMs(ms);
	LedOff(pin);
}

void LedToggle(uint32_t pin) {
	GPIOD->ODR ^= BV(pin);
}
