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
	ADCn_Ch_e raw_gyro_port, raw_angle_port, raw_accel_port;
	int16 raw_gyro;
	int16 raw_angle;
	int16 raw_z;
	int16 raw_setpoint;
	int16 raw_offset;


public:
	balance_gyro(ADCn_Ch_e, ADCn_Ch_e, ADCn_Ch_e, int16);
	void refresh();
	int16 get_raw_gyro();
	int16 get_raw_angle();
	int16 get_offset();
	int16 get_accel();


};

#endif /* BALANCE_GYRO_H_ */
