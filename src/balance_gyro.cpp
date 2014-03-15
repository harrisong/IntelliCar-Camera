/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_gyro.h>

balance_gyro::balance_gyro(ADCn_Ch_e p1, ADCn_Ch_e p2, ADCn_Ch_e p3, int16 sp){
	raw_gyro_port = p1;
	raw_angle_port = p2;
	raw_accel_port = p3;
	adc_init(raw_gyro_port);
	adc_init(raw_angle_port);
	adc_init(raw_accel_port);

	raw_setpoint = sp;
	raw_gyro = 0;
	refresh();
}

void balance_gyro::refresh(){
	totalsample++;
	raw_gyro = ((int16) adc_once(raw_gyro_port, ADC_16bit) + raw_gyro)/totalsample;
	raw_angle = (int16) adc_once(raw_angle_port, ADC_16bit);
	raw_offset = raw_angle - raw_setpoint;

	raw_z =  (int16)adc_once(raw_accel_port, ADC_16bit);
}

int16 balance_gyro::get_raw_gyro(){
	return raw_gyro;
}

int16 balance_gyro::get_raw_angle(){
	return raw_angle;
}

int16 balance_gyro::get_offset(){
	return raw_offset;
}

int16 balance_gyro::get_accel(){
	return raw_z;
}

