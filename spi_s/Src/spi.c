/*
 * spi.c
 *
 *  Created on: Apr 1, 2025
 *      Author: admin
 */

#include "spi.h"

void SpiInit(void) {
	// config PE.3 as GPIO
	// clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// gpio mode is output (01)
	GPIOE->MODER |= BV(3*2);
	GPIOE->MODER &= ~BV(3*2+1);
	// gpio output data push-pull
	GPIOE->OTYPER &= ~BV(3);
	// gpio output speed low
	GPIOE->OSPEEDR &= ~(BV(3*2) | BV(3*2+1));
	// disable pull-up and pull-down resistor
	GPIOE->PUPDR &= ~(BV(3*2) | BV(3*2+1));

	// config PA5, PA6, PA7 as SPI1
	// gpio config
	// clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// gpio mode alt fn (10)
	GPIOA->MODER |= BV(5*2+1) | BV(6*2+1) | BV(7*2+1);
	GPIOA->MODER &= ~(BV(5*2) | BV(6*2) | BV(7*2));
	// set alt fn = SPI1 = AF5
	GPIOA->AFR[0] |= (5 << (5*4)) | (5 << (6*4)) | (5 << (7*4));
	// disable pull-up and pull-down resistor
	GPIOA->PUPDR &= ~(BV(5*2+1) | BV(6*2+1) | BV(7*2+1) | BV(5*2) | BV(6*2) | BV(7*2));

	// spi config
	// enable spi clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// config spi in CR1 -- master mode, software slave mgmt, set bit rate (010 = 2Mhz), cpol=0, cpha=0, lsbf=0,
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BR_1;
	// disable spi intrs & dma (default), frame format = spi
	SPI1->CR2 = 0x0000;
	// enable spi in CR1
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SpiCSEnable(void) {
	// write 0
	GPIOE->ODR &= ~BV(3);
}

void SpiCSDisable(void) {
	// write 1
	GPIOE->ODR |= BV(3);
}

uint16_t SpiTransfer(uint16_t data) {
	// wait until tx regr is not empty
	while(!(SPI1->SR & SPI_SR_TXE));
	// write data into dr
	SPI1->DR = data;
	// wait until data is received
	while(!(SPI1->SR & SPI_SR_RXNE));
	// return the recieved data
	return SPI1->DR;
}

void SpiTransmit(uint16_t data) {
	SpiTransfer(data);
}

uint16_t SpiReceive(void) {
	uint16_t data = SpiTransfer(0x00);
	return data;
}

void SpiWrite(uint8_t internalAddr, uint8_t data[], uint8_t size) {
	SpiCSEnable();
	internalAddr &= ~BV(7);
	SpiTransmit(internalAddr);
	for(int i=0; i<size; i++)
		SpiTransmit(data[i]);
	SpiCSDisable();
}

void SpiRead(uint8_t internalAddr, uint8_t data[], uint8_t size) {
	SpiCSEnable();
	internalAddr |= BV(7);
	SpiTransmit(internalAddr);
	for(int i=0; i<size; i++)
		data[i] = SpiReceive();
	SpiCSDisable();
}


