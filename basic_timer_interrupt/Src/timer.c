/*
 * timer.c
 *
 *  Created on: Sep 29, 2025
 *      Author: krushna
 */


#include"timer.h"
#include "led.h"

void TimerInit(uint32_t ms)
{
	//enable timer APB clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	//SET THE PRESCALAR
	TIM6->PSC=PR-1;
	//CALCULATE NUMBER OF CLOCKS TO COUNT
	uint32_t cnt=(FPCLK/1000) * ms / PR;
	//set max count in ARR
	TIM6->CNT =0;
	//enable the timer interrupt in peripheral
	TIM6->DIER |= TIM_DIER_UIE;
	//enable the timer interrupt in nvic
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	//enable tmer clock (mandatory )
	TIM6->CR1 |= TIM_CR1_CEN;
}
//implement Interrupt Handler for TIM6
void TIM6_DAC_IRQHandler(void)
		{
	      //check if TIM6 interrupt occured
	if(TIM6->SR & TIM_SR_UIF)
	{
		//HANDLE TIM6 INTERRUPT -TOOGLE LED
		led_toggle(LED_BLUE);
		//clear /ACK TIM6 interrupt
		TIM6->SR &= ~TIM_SR_UIF;

	}
		}


