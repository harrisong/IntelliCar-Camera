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



class BalanceGyro{
private:
	ADCn_Ch_e raw_gyro_port, raw_angle_port, raw_z_port, raw_x_port;
	float raw_gyro;
	int16 raw_angle;
	float Rz, Rx, accel;
	int16 raw_setpoint, raw_offset, old_raw_offset, omega;

	int totalsample;
	
	float Vmax;
	int Adc16max;
	float Gyrozero;
	float Gyroscale;
	float Accelzero;
	float Accelscale;


public:
	BalanceGyro(ADCn_Ch_e, ADCn_Ch_e, ADCn_Ch_e, ADCn_Ch_e, int16);
	void refresh();
	float get_raw_gyro();
	int16 get_raw_angle();
	int16 get_offset();
	int16 get_omega();
	float get_accel();


};

#endif /* BALANCE_GYRO_H_ */