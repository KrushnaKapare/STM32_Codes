/*
 * clock.c
 *
 *  Created on: Oct 1, 2025
 *      Author: krushna
 */


#include "clock.h"

void Clock_Setup(void)
{
	//Enable HSE (CR bit = 16)
	RCC->CR |= RCC_CR_HSEON;
	//WAIT TILL HSE IS READY (CR BIT = 17)
	while(!(RCC->CR & RCC_CR_HSERDY));

	//ENABLE POWER INTERFACE CLOCK (APB1ENR BIT = 28)

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	/* set voltage scale to 1 for max frequency (PWR_CR:bit 14)
	 * (0b10) scale 2 for fCLK <= 144 Mhz
	 * (0b01) scale 3 for 144 Mhz < fCLK <= 168 Mhz
	 */
#if CCLK <= 144000000UL
	PWR->CR |= 2 << PWR_CR_VOS_Pos;
#elif CCLK > 144000000UL
	PWR->CR |= 1 << PWR_CR_VOS_Pos;
#endif

	/* set AHB prescaler to /1 (CFGR:bits 7:4) */
	RCC->CFGR |= 0 << RCC_CFGR_HPRE_Pos;
	// set AHB prescalar (CFGR:bits 7:4)
#if PRE_H == 2
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2;
#elif PRE_H == 4
	RCC->CFGR |= RCC_CFGR_HPRE_DIV4;
#elif PRE_H == 8
	RCC->CFGR |= RCC_CFGR_HPRE_DIV8;
#elif PRE_H == 16
	RCC->CFGR |= RCC_CFGR_HPRE_DIV16;
#elif PRE_H == 64
	RCC->CFGR |= RCC_CFGR_HPRE_DIV64;
#elif PRE_H == 128
	RCC->CFGR |= RCC_CFGR_HPRE_DIV128;
#elif PRE_H == 256
	RCC->CFGR |= RCC_CFGR_HPRE_DIV256;
#elif PRE_H == 512
	RCC->CFGR |= RCC_CFGR_HPRE_DIV512;
#endif

	/* set APB low speed prescaler to /4 (APB1) (CFGR:bits 12:10) */
#if PRE_P1 == 2
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
#elif PRE_P1 == 4
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
#elif PRE_P1 == 8
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;
#elif PRE_P1 == 16
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;
#endif
	// set APB high speed prescaler to /2 (APB2) (CFGR:bits 15:13) */
#if PRE_P2 == 2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
#elif PRE_P2 == 4
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
#elif PRE_P2 == 8
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV8;
#elif PRE_P2 == 16
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV16;
#endif


	/* Set M, N, P and Q PLL dividers
	 * PLLCFGR: bits 5:0 (M), 14:6 (N), 17:16 (P), 27:24 (Q)
	 * Set PLL source to HSE, PLLCFGR: bit 22, 1:HSE, 0:HSI
	 */
	RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos)
					| (PLL_N << RCC_PLLCFGR_PLLN_Pos)
					| (((PLL_P / 2) -1) << RCC_PLLCFGR_PLLP_Pos)
					| (PLL_Q << RCC_PLLCFGR_PLLQ_Pos)
					| RCC_PLLCFGR_PLLSRC_HSE;
	// Enable the main PLL (CR: bit 24)
	RCC->CR |= RCC_CR_PLLON;
	// Wait till the main PLL is ready (CR: bit 25)
	while(!(RCC->CR & RCC_CR_PLLRDY));
	/* Configure Flash
	 * prefetch enable (ACR:bit 8)
	 * instruction cache enable (ACR:bit 9)
	 * data cache enable (ACR:bit 10)
	 * set latency to 5 wait states (ARC:bits 2:0)
	 */
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | (FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);

	/* Select the main PLL as system clock source, (CFGR:bits 1:0)
	 * 0b00 - HSI
	 * 0b01 - HSE
	 * 0b10 - PLL
	 */
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_1;
	//Wait till the main PLL is used as system clock source (CFGR:bits 3:2)
	while (!(RCC->CFGR & RCC_CFGR_SWS_1));

	// update SystemCoreClock variable
	SystemCoreClock = CCLK;
}
