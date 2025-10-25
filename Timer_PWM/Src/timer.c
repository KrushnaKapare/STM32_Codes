/*
 * timer.c
 *
 *  Created on: Sep 30, 2025
 *      Author: krushna
 */

#include "timer.h"

#include "stm32f407xx.h"
//#define BV(n) (1<<(n))
void TimerPwmInit(void)
{

	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		// set PC6 mode as Alt Fn (10)
		GPIOC->MODER |= BV(2 * 6 + 1);
		GPIOC->MODER &= ~BV(2 * 6);
		// disable pull-up and pull-down regrs
		GPIOC->PUPDR &= ~(BV(2 * 6) | BV(2 * 6 + 1));
		// set alt fn "3" as TIM8
		GPIOC->AFR[0] |= (3 << (6 * 4));



	//PWM
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	TIM8->PSC =TIM_PR -1;
	TIM8->ARR =100 -1;
	TIM8->CCR1 =0;


	TIM8->CCMR1 &= ~(TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1);
	TIM8->CCER &= ~TIM_CCER_CC1P;

	TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM8->CR1 |= TIM_CR1_ARPE;

	TIM8->CR1 &= ~(TIM_CR1_CMS_0 | TIM_CR1_CMS_1);

	TIM8->CCER |= TIM_CCER_CC1E;

	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CR1 |= TIM_CR1_CEN;



}
