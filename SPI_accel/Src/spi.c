/*
 * spi.c
 *
 *  Created on: Sep 21, 2025
 *      Author: krushna
 */


#include "spi.h"

void spi_init(void)
{
	//config PE3 as GPIO
	//clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	//GPIO MODE IS OUTPUT (01)
	GPIOE->MODER |= BV(3*2);
	GPIOE->MODER &= ~BV(3*2+1);
	//GPIO OUTPUT DATA PUSH PULL
	GPIOE->OSPEEDR &= ~BV(3);
	//GPIO OUTPUT SPEED LOW
	GPIOE->OSPEEDR &= ~(BV(3*2) | BV(3*2+1));

	//DISABLE PULL-UP AND PULL-DOWN RESISTOR
	GPIOE->PUPDR &= ~(BV(3*2) | BV(3*2+1));


	//CONFIG PA5,PA6,PA7 AS SPI1
	//GPIO CONFIG
	//CLOCK ENABLE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//GPIO MODE ALT FN (10)
	GPIOA->MODER |= BV(5*2+1) | BV(6*2+1) | BV(7*2+1);
	GPIOA->MODER &= ~(BV(5*2) |BV (6*2) |BV(7*2));
	//SET ALT FN =SPI =AF5
    GPIOA->AFR[0] |= (5<<(5*4)| (5<<(6*4)) |(5<<7*4));

    //DISBALE PULL-UP AND PULL DOWN RSISTOR
    GPIOA->PUPDR &= ~(BV(5*2+1) |BV(6*2+1) |BV(7*2+1) | BV(5*2) | BV(6*2) | BV(7*2));

    //SPI CONFIG
    //ENABLE SPI CLOCK
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    //CONFIG SPI IN CR1 -- MASTER MODE , SOFTWARE SLAVE MGMT ,SET BIT RATE (010=2MHZ),CPOL=0,CPHA=0, LSBF=0
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BR_1;

    //DISABLE SPI INTER & DMA (DEFAULT) , FRAME FORMAT =SPI
    SPI1->CR2 =0x0000;
    //ENABLE SPI IN CR1
    SPI1->CR1 |= SPI_CR1_SPE;



}


void spi_cs_enable(void)
{
	//write = 0
	GPIOE->ODR &= ~BV(3);


}

void spi_cs_disable(void)
{

	//write= 1
	GPIOE->ODR |= BV(3);
}

uint16_t spi_transfer(uint16_t data)
{
	//wait until tx regr is not empty
	while(!(SPI1->SR & SPI_SR_TXE));

	//WRITE DATA INTO DR
	SPI1->DR = data;
	//WAIT UNTIL DATA IS RECEIVED
	while(!(SPI1->SR & SPI_SR_RXNE));
	//RETURN THE RECEIEVED DATA
	return SPI1->DR;

}

void spi_transmit (uint16_t data )
{
	spi_transfer(data);

}

uint16_t spi_receive(void)
{
	uint16_t data =spi_transfer(0x00);
	return data;
}

void spi_write(uint8_t internal_addr,uint8_t data[],uint8_t size)
{
	spi_cs_enable();
	internal_addr &= ~BV(7);
	spi_transmit(internal_addr);
	for(int i=0; i<size;i++)
		spi_transmit (data[i]);
	spi_cs_disable();
}

void spi_read(uint8_t internal_addr, uint8_t data[],uint8_t size)
{
	spi_cs_enable();
	internal_addr |= BV(7);
	spi_transmit(internal_addr);
	for(int i=0 ;i<size ; i++)
		data[i]= spi_receive();
	spi_cs_disable();

}






















