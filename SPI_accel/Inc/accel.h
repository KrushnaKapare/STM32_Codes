/*
 * accel.h
 *
 *  Created on: Sep 21, 2025
 *      Author: krushna
 */

#ifndef ACCEL_H_
#define ACCEL_H_


#include "spi.h"

typedef struct accel_data{
	int16_t x;
	int16_t y;
	int16_t z;

}accel_data_t;

#define ACCEL_CR4	0x20
#define ACCEL_STATUS 0x27
#define ACCEL_X 0x28
#define ACCEL_Y 0x2A
#define ACCEL_Z 0x2C

#define CR4_ODR_25 BV(6)
#define CR4_X_EN BV(0)
#define CR4_Y_EN BV(1)
#define CR4_Z_EN BV(2)

#define CR4_XYZ_EN (CR4_X_EN | CR4_Y_EN | CR4_Z_EN)

#define STATUS_X_DA BV(0)
#define STATUS_Y_DA BV(1)
#define STATUS_Z_DA BV(2)
#define STATUS_XYZ_DA BV(3)

void accel_init(void);
int accel_wait_for_change(void);
void accel_read(accel_data_t *data);




#endif /* ACCEL_H_ */
