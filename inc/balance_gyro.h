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
#include <math.h>
//#include <cstdint>
//#include <libutil/clock.h>



class BalanceAccel{
private:
	ADCn_Ch_e raw_x_port;

	float raw_accel_angle;

	float VR;
	float Rx, Rz;
	float Vmax;
	int Adc16max;

	float Accelzero;
	float Accelscale;

public:
	BalanceAccel(ADCn_Ch_e);
	void Refresh();
	float GetRawAngle(){
		return raw_accel_angle;
	}


};

#endif /* BALANCE_GYRO_H_ */
