/*
 * timer.c
 *
 *  Created on: Sep 29, 2025
 *      Author: krushna
 */

#include "timer.h"

void TimerInit(void)
{
	//enable timer APB clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	//SET THE PRESCALAR
	TIM6->PSC =PR-1;



}

void TimerDelayMs(uint32_t ms)
{
	//calculate number of clock to count
	uint32_t cnt =(FPCLK / 1000) * ms /PR;
	//set max count in ARR
	TIM6->ARR =cnt -1;
	//start counting from zero
	TIM6->CNT =0;
	//ENABLE TIMER CLOCK (MANDATORY)
	TIM6->CR1 |= TIM_CR1_CEN;
	//WAIT FOR UPDATE
	while(!(TIM6->SR & TIM_SR_UIF));
	//clear update flag in SR
	TIM6->SR &= ~TIM_SR_UIF;
	//STOP TIMER CLOCK
	TIM6->CR1 &= ~TIM_CR1_CEN;

}
