/*
 * led.h
 *
 *  Created on: Sep 13, 2025
 *      Author: krushna
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f407xx.h"
#define BV(n) (1 << (n))
#define GPIOD_CLK_EN 3

#define LED_GPIO   GPIOD

#define LED_GREEN   12
#define LED_ORANGE  13
#define LED_RED     14
#define LED_BLUE    15


/*
 * uint8_t is for 8 bits >>>>> 1 byte which is 8 bits here we required 4 bits only
 * uint16_t is for 16 bits which is 2 bytes
   uint32_t is for 32 bits which is 4 bytes


   which is internally converted into this
   ypedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
 */
void led_init(uint8_t pin);
void led_on(uint8_t pin);
void led_off(uint8_t pin);
void led_toggle(uint8_t pin);


#endif /* LED_H_ */
