/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
//#include <cstdlib>
#include <balance_gyro.h>
#include <libsc/com/config.h>
//#include <FIRE_MK60_conf.h>
//#include <float.h>

//#include <mini_common.h>
//#include <hw_common.h>
//#include <MK60_pit.h>
//#include <MK60_gpio.h>
//#include <MK60_i2c.h>



BalanceAccel::BalanceAccel(ADCn_Ch_e p3):
	raw_x_port(p3),
	raw_accel_angle(90),
	Rx(0),
	Vmax(3.3), Adc16max(65535),
	Accelzero(1.65f), Accelscale(0.206) {

	adc_init(raw_x_port);
	adc_init(RZADC);
}
void BalanceAccel::Refresh(){

		Rx =  (((float) adc_once(raw_x_port, ADC_10bit) * Vmax/ 1024) - Accelzero);
		Rx *= 1.25f;
		Rx -= -0.005f;

		if(Rx > 1.0){
			Rx = 1.0;
		}else if(Rx < -1.0){
			Rx = -1.0;
		}

		Rz = (((float) adc_once(RZADC, ADC_10bit) * Vmax/ 1024) - Accelzero);
		Rz *= 1.25f;
		Rz -= -0.005f;

		if(Rz > 1.0){
			Rz = 1.0;
		}else if(Rz < -1.0){
			Rz = -1.0;
		}

		if(sqrt(Rx * Rx + Rz * Rz) > 0.7f){
			raw_accel_angle = 90 - (acos(Rx) * 180 / 3.1415f - 90);
		}

}



