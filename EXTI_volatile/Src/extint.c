/*
 * extint.c
 *
 *  Created on: Sep 16, 2025
 *      Author: krushna
 */

#include"led.h"

volatile int ext_flag =0;

void extint_init(void)
{
	// ENABLE CLOCK FOR GPIOA
	RCC->AHB1ENR |= BV(0);

	//CONFIGURE GPIOA0 AS INPUT
	GPIOA->MODER &= ~(BV(0) | BV(1));
	GPIOA->PUPDR &= ~(BV(0) | BV(1));

	//CONFIGURE EXTERNAL INTERUUPT EXTI0 INTO SYSTEM CONFIGURATION

	SYSCFG->EXTICR[0] &= ~(BV(0)|BV(1)|BV(2)|BV(3));

	//CONFIGURE EXTERNAL INTERRUPT EXTI0 INTO EXTERNAL CIRCUIT
	EXTI->IMR |= BV(0);
	EXTI->RTSR |= BV(0);

	// CONFIGURE EXTERNAL INTERRUPTS EXTI0 INTO NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);

}

void EXTI0_IRQHandler(void)
{

	// clear pending bits of EXTI0 in PR
	EXTI->PR |= BV(0);

	//TAKE ACTION
	led_toggle(13);

}






