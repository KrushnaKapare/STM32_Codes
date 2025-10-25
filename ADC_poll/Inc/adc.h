/*
 * adc.h
 *
 *  Created on: Oct 1, 2025
 *      Author: krushna
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"


void AdcInit(void);
uint16_t AdcRead(void);

#endif /* ADC_H_ */
