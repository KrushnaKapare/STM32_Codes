/*
 * wdt.c
 *
 *  Created on: Sep 30, 2025
 *      Author: krushna
 */

#include "wdt.h"

void WdtInit(uint32_t ms)
{
	RCC->CSR |= RCC_CSR_LSION;
	while(!(RCC->CSR & RCC_CSR_LSIRDY))
		;
	IWDG->KR = 0xCCCC;
	IWDG->KR = 0x5555;
	IWDG->PR =WDT_PR_VAL;

	uint32_t cnt =(WDT_LSICLK /1000) * ms / WDT_PR;
	IWDG->RLR = cnt -1;
	while(IWDG->SR != 0)
		;
	WdtFeed();

}

void WdtFeed(void)
{
	IWDG->KR = 0xAAAA;

}
int IsWdtReset(void)
{
	if((RCC->CSR & RCC_CSR_IWDGRSTF) != 0)
		return 1;
	return 0;

}
