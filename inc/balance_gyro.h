/*
 * balance_gyro.h
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */

#ifndef BALANCE_GYRO_H_
#define BALANCE_GYRO_H_

#include <mini_common.h>
#include <MK60_adc.h>



class balance_gyro{
private:

public:
	balance_gyro(ADCn_Ch_e);
	int16 get_gyro();
	int16 get_angle();


};

#endif /* BALANCE_GYRO_H_ */
