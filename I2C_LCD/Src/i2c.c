/*
 * i2c.c
 *
 *  Created on: Sep 18, 2025
 *      Author: krushna
 */

#include "i2c.h"

void I2CInit(void)
{
	//gpio config
	//enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//SET MODE AS ALT FUNCTION (10)
	GPIOB->MODER |= BV(2*6+1) | BV(2*7+1);
	GPIOB->MODER &= ~(BV(2*6) | BV(2*7));

	//SET ALT FN TO AF4 (I2C)
	GPIOB->AFR[0]=(4 << (4*6)) | (4<<(4*7));

	//NO PULL UP & PULL DOWN REGR
	GPIOB->PUPDR &= ~(BV(2*6+1) |BV(2*7+1) | BV(2*6)| BV(2*7));

	//ENABLE OPEN-DRAIN FOR PB6 &PB7
	GPIOB->OTYPER |= BV(6)|BV(7);

	//I2C CONFIG
	//ENABLE I2C PERI CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	//I2C SW RESET
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 = 0;  //CLEAR ALL CR1 BITS

	//PERI CLOCK -- CR2 =16MHz
	I2C1->CR2 |= 16 << I2C_CR2_FREQ_Pos;

	//SET I2C CLOCK  -- CCR =80 (STD MODE=100KHz)
	I2C1->CCR =80;
	I2C1->CCR &= ~I2C_CCR_FS;  //STANDARD MODE (DEFAULT)

	//SET TRISE -- TRISE=17
	I2C1->TRISE =17;

	//ENABLE ACK
	I2C1->CR1 |= I2C_CR1_ACK;

	//ENABLE I2C PERI
	I2C1->CR1 |= I2C_CR1_PE;

}

void I2CStart(void)
{
	//send start bit
	I2C1->CR1 |= I2C_CR1_START;

	//WAIT FOR START BIT SENT ON BUS
	while(!(I2C1->SR1 & I2C_SR1_SB));

}

void I2CRepeatStart(void)
{
	I2CStart();
}

void I2CStop(void)
{
	//send stop bit
	I2C1->CR1 |= I2C_CR1_STOP;

	//WAIT FOR STOP BIT SENT ON BUS
	while (I2C1->SR2 & I2C_SR2_BUSY);

}

void I2CSendSlaveAddr(uint8_t addr)
{
	//write slave addr in DR
	I2C1->DR= addr;
	//wait until slave addr is sent
	while(!(I2C1->SR1 & I2C_SR1_ADDR));

	//READ STATUS REGR TO CLEAR ACKS
	(void)I2C1->SR1;
	(void)I2C1->SR2;


}

void I2CSendData(uint8_t data)
{
	//wait until data is sent
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	//WRITE DATA IN DR
	I2C1->DR =data;
	//POLL FOR BTF IS  TRANSFERRED
	while(!(I2C1->SR1 &I2C_SR1_BTF));

}

uint8_t I2CRecvDataAck(void)
{
	//send ack for next byte read
	I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;

	//WAIT UNTIL DATA IS RECIVE
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	//COLLECT RECEIVED DATA AND RETURN IT
	return I2C1->DR;

}

uint8_t I2CRecvDataNAck(void)
{

	//send no ack for next byte read
	I2C1->CR1&= ~(I2C_CR1_ACK | I2C_CR1_POS);

	//WAIT UNTIL DATA IS RECEIVED
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	//COLLECT RECEIVED DATA AND RETURN IT
	return I2C1->DR;
}

void I2CWrite(uint8_t addr,uint8_t data)
{
	I2CStart();
	I2CSendSlaveAddr(addr);
	I2CSendData(data);
	I2CStop();

}























