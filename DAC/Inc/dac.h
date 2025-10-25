/*
 * dac.h
 *
 *  Created on: Apr 8, 2025
 *      Author: admin
 */

#ifndef DAC_H_
#define DAC_H_

#include "stm32f4xx.h"

void DacInit(void);
void DacSetValue(uint16_t val);

#endif /* DAC_H_ */
