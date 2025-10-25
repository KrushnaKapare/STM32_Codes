/*
 * accel.c
 *
 *  Created on: Apr 2, 2025
 *      Author: admin
 */

#include "accel.h"

void AccelInit(void) {
	// init spi
	SpiInit();
	// power on accel with all 3 axis enabled
	uint8_t val[1];
	val[0] = CR4_ODR_25 | CR4_XYZ_EN;
	SpiWrite(ACCEL_CR4, val, 1);
}

int AccelWaitForChange(void) {
	uint8_t val[1];
	do {
		SpiRead(ACCEL_STATUS, val, 1);
	} while(!(val[0] & STATUS_XYZ_DA));
	return 1;
}

void AccelRead(AccelData_t *data) {
	uint8_t val[2];
	// read x axis values (xl & xh)
	SpiRead(ACCEL_X, val, 2);
	// combine them to make 16 bit x value
	data->x = val[0] | ((int16_t)val[1] << 8);

	// read y axis values (yl & yh)
	SpiRead(ACCEL_Y, val, 2);
	// combine them to make 16 bit y value
	data->y = val[0] | ((int16_t)val[1] << 8);

	// read z axis values (zl & zh)
	SpiRead(ACCEL_Z, val, 2);
	// combine them to make 16 bit z value
	data->z = val[0] | ((int16_t)val[1] << 8);
}

