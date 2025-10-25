/*
 * timer.c
 *
 *  Created on: Sep 30, 2025
 *      Author: krushna
 */

#include "timer.h"

void TimerInit(void)
{
	// GPIO (GPIOD 12-15) config
		// enable GPIO clock
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		// set mode to alt fn i.e. 10
		GPIOD->MODER |= BV(12*2+1) | BV(13*2+1) | BV(14*2+1) | BV(15*2+1);
		GPIOD->MODER &= ~(BV(12*2) | BV(13*2) | BV(14*2) | BV(15*2));
		// disable pull-up/pull-down resistors
		GPIOD->PUPDR &= ~(BV(12*2) | BV(13*2) | BV(14*2) | BV(15*2) | BV(12*2+1) | BV(13*2+1) | BV(14*2+1) | BV(15*2+1));
		// set alt fn to AF2 (TIM4 Output Channels)
		GPIOD->AFR[1] |= BV(17) | BV(21) | BV(25) | BV(29);


		//Timer (TIM4) config
		//0 enable timer peri clock
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		//select the counter clock ("internal")
		TIM4->PSC=PR-1;
		//2.WRITE THE DESIRED DATA IN THE TIMx_ARR AND TIMx_CCRx REGISTERS
		TIM4->ARR =ARR_VAL -1;
		TIM4->CCR1 = CCR1_VAL -1;
		TIM4->CCR2 = CCR2_VAL -1;
		TIM4->CCR3 = CCR3_VAL -1;
		TIM4->CCR4 = CCR4_VAL -1;

		TIM4->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_0| TIM_CCMR1_OC2M_1;
		TIM4->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_0| TIM_CCMR2_OC4M_1;

		TIM4->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		TIM4->CR1 |= TIM_CR1_CEN;
}
