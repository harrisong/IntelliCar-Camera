/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_gyro.h>

balance_gyro::balance_gyro(ADCn_Ch_e port){
	adc_init(port);
}



