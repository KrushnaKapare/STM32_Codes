/*
 * uart.h
 *
 *  Created on: Sep 17, 2025
 *      Author: krushna
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f407xx.h"

/* USART2
 * PA2 = TX
 * PA3 = RX
 *
 */

void UartInit(uint32_t baud);
void UartPutch(int ch);
int UartGetch(void);
void UartPuts(char *str);
void UartGets(char *str);






#endif /* UART_H_ */
