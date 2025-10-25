/*
 * dma.c
 *
 *  Created on: Oct 1, 2025
 *      Author: krushna
 */

#include "dma.h"

void DMA_Init(void)
{
	RCC->APB1ENR |= RCC_AHB1ENR_DMA2EN;

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

void DMA_Mem_Transfer(uint8_t *dma_mem_dest,uint8_t *dma_mem_src,uint16_t size )
{
	DMA2_Stream0->CR =0x0000;
	while(DMA2_Stream0->CR & DMA_SxCR_EN);

	DMA2_Stream0->PAR = (uint32_t)dma_mem_src;
	DMA2_Stream0->M0AR = (uint32_t)dma_mem_dest;

	DMA2_Stream0->NDTR =size;

	DMA2_Stream0->CR |= (0<<DMA_SxCR_CHSEL_Pos);
	DMA2_Stream0->CR |= DMA_SxCR_MINC;
	DMA2_Stream0->CR |= DMA_SxCR_PINC;
	DMA2_Stream0->CR |= (0<<DMA_SxCR_MSIZE_Pos);
	DMA2_Stream0->CR |= (0<<DMA_SxCR_PSIZE_Pos);
	DMA2_Stream0->CR |= DMA_SxCR_PL_1;
	DMA2_Stream0->CR |= DMA_SxCR_TCIE;

	DMA2_Stream0->CR |= DMA_SxCR_EN;

}

volatile int dma2_flag = 0;
void DMA2_Stream0_IRQHandler(void)
{
	if(DMA2->LISR & DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		dma2_flag =1;
	}
}



