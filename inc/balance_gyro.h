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
#include <cstdint>
#include <libutil/clock.h>



class BalanceGyro{
private:
	ADCn_Ch_e raw_gyro_port, raw_angle_port, raw_z_port, raw_x_port;
	float raw_gyro, raw_gyro_angle, raw_setpoint;
	float raw_accel_angle;

	float kalman_angle, comp_angle;
	float VR;
	float Rz, Rx, R;
	float raw_offset, old_raw_offset, omega;



	int totalsample;
	
	float Vmax;
	int Adc16max;
	float Gyrozero;
	float Gyroscale;
	float Accelzero;
	float Accelscale;
	int count = 0;
	uint16_t c_time, p_time, d_time, p_time_2, d_time_2;



public:
	float init_angle;
	BalanceGyro(ADCn_Ch_e, ADCn_Ch_e, ADCn_Ch_e, int16);
	void Refresh();
	float get_raw_gyro();
	float GetRawAngle();
	float GetOffset();
	float GetOmega();
	float get_accel();
	void ChangeSetPoint(uint16);


};

#endif /* BALANCE_GYRO_H_ */
