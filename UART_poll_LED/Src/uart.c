/*
 * uart.c
 *
 *  Created on: Sep 17, 2025
 *      Author: krushna
 */

#include "uart.h"
#include"led.h"
#include <string.h>
//#define BV(n) (1<<(n))

void UartInit(uint32_t baud)
{
	/* GPIO config */
	//enable gpio clock
	// RCC-> AHB1ENR |= BV (RCC_AHB1ENR_GPIOAEN_Pos);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//SET GPIO - NO PULL UP & PULL DOWN
	GPIOA->PUPDR &= ~(BV(2*2+1) | BV(2*2) |BV(2*3+1) |BV(2*3));

	//SET GPIO (PA2 , PA3 AS ALT FUNCTION =10
	GPIOA->MODER |= BV(2*2+1) | BV(2*3+1);
	GPIOA->MODER &= ~(BV(2*2) | BV(2*3));

	//SET GPIO (PA2,PA3) PIN ALT FN AS UART (AF7) --- AFRL =AFR[0],AFRH =AFR[1]
	GPIOA->AFR[0]=(7 << (2*4)) |(7 << (3*4));
	/*UART CONFIG */
	//ENABLE UART2 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	//UART2  CONFIG (8N1) = 8-bit DATA TRANSFER=0, DISABLE PARITY=0, tx=en1, rx en=1
	USART2->CR1 =USART_CR1_TE | USART_CR1_RE;

	//baud rate setting
	if (baud == 9600)
		USART2->BRR = 0x0683;
	else if (baud == 38400)
		USART2->BRR =0x01A1;
	else if(baud== 115200)
		USART2->BRR = 0x008B;
	//enable uart=1
	USART2->CR1 |= USART_CR1_UE;
}

void UartPutch(int ch )
{
	//write char in TDR
	USART2->DR =ch;

	//wait until Tx is done (until Tx regr is not empty)
	while((USART2->SR & USART_SR_TXE) == 0)
		;
}

int UartGetch(void)
{
	//wait until data is Rx is done (until Rx regr is empty)
	while ((USART2-> SR & USART_SR_RXNE) == 0)
		;
	//read char from RDR
	return USART2->DR;

}

void UartPuts(char *str)
{
	for (int i=0; str[i] != '\0';i++)
		UartPutch(str[i]);
}

void UartGets(char *str)
{
	int i=0;
	char ch;
	do{
		ch =UartGetch();
		str[i]=ch;
		i++;
	}while(ch != '\r');
	str[i] = '\n';
	i++;
	str[i]='\0';

	uint8_t arr[] = {LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE};
				for(uint8_t i = 0 ; i < 4 ; i++)
					led_init(arr[i]);
				char *ch1="ON";
	if(str==ch1)
			{
				led_toggle(arr[1]);
			}
}



