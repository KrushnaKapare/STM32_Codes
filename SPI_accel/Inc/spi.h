/*
 * spi.h
 *
 *  Created on: Sep 21, 2025
 *      Author: krushna
 */

#ifndef SPI_H_
#define SPI_H_

#include"stm32f4xx.h"

void  spi_init(void);
void spi_cs_enable(void);
void spi_cs_disable(void);
uint16_t spi_transfer(uint16_t data);
void spi_transmit(uint16_t data);
uint16_t spi_receive(void);
void spi_write(uint8_t internal_addr,uint8_t data[],uint8_t size);
void spi_read(uint8_t internal_addr, uint8_t data[],uint8_t size);



#endif /* SPI_H_ */
