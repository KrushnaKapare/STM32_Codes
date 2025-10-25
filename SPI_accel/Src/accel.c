/*
 * accel.c
 *
 *  Created on: Sep 22, 2025
 *      Author: krushna
 */


#include "accel.h"

void accel_init (void)
{
	//init spi
	spi_init();
	//power on accel with all 3 axis enabled
	uint8_t val[1];
	val[0]= CR4_ODR_25 | CR4_XYZ_EN;
	spi_write(ACCEL_CR4,val,1);


}

int accel_wait_for_change(void)
{
	uint8_t val[1];
	do {
		spi_read(ACCEL_STATUS,val,1);
	}while(!(val[0] & STATUS_XYZ_DA));
	return 1;
}

void accel_read(accel_data_t *data)
{
	uint8_t val[2];
	//read x axis values (xl & xh)
	spi_read(ACCEL_Y ,val,2);
	//combine them to make 16 bit x value
	data->x =val[0] | ((uint16_t )val[1]<<8);


	//read y axis values (yl & yh)
	spi_read(ACCEL_Y, val,2);

	//combine them to make 16 bit y value
	data->y = val[0] | ((uint16_t ) val[1] <<8);

	//read z-axis values (zl & zh)
	spi_read(ACCEL_Z,val,2);
	//combine them to make 16 bit z value
	data->z = val[0] | ((uint16_t ) val[1]<<8);


}
