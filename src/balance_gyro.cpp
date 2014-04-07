/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_gyro.h>

BalanceGyro::BalanceGyro(ADCn_Ch_e p1, ADCn_Ch_e p2, ADCn_Ch_e p3, ADCn_Ch_e p4, int16 sp):
	raw_gyro_port(p1), raw_angle_port(p2), raw_z_port(p3), raw_x_port(p4),
	raw_setpoint(sp), raw_gyro(0), raw_offset(0), old_raw_offset(0), omega(0),
	Vmax(3.3), Adc16max(65535), Gyrozero(1.23), Gyroscale(0.00067), Accelzero(1.65), Accelscale(0.4785) {

	//adc_init(raw_gyro_port);
	adc_init(raw_angle_port);
	//adc_init(raw_z_port);
	//adc_init(raw_x_port);
	
	Refresh();
}

void BalanceGyro::Refresh(){
	totalsample++;
	raw_angle = (int16) adc_once(raw_angle_port, ADC_16bit);
	raw_offset = raw_angle - raw_setpoint;
	omega = raw_offset - old_raw_offset;
	old_raw_offset = raw_offset;

	//raw_gyro = ((int16) adc_once(raw_gyro_port, ADC_16bit) + raw_gyro)/totalsample;
	////Use Kalman filter now, no need values for complementary filter////
	/*raw_gyro = (adc_once(raw_gyro_port, ADC_16bit) * Vmax / Adc16max - Gyrozero) / Gyroscale;
	
	Rx =  (adc_once(raw_x_port, ADC_16bit) * Vmax / Adc16max - Accelzero) / Accelscale;
	Rz =  (adc_once(raw_z_port, ADC_16bit) * Vmax / Adc16max - Accelzero) / Accelscale;
	accel = sqrt(pow(Rx,2) + pow(Rz,2));*/
}

float BalanceGyro::get_raw_gyro(){
	return raw_gyro;
}

int16 BalanceGyro::GetRawAngle(){
	return raw_angle;
}

int16 BalanceGyro::GetOffset(){
	return raw_offset;
}

int16 BalanceGyro::GetOmega(){
	return omega;
}

float BalanceGyro::get_accel(){
	return accel;
}

