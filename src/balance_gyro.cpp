/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
//#include <cstdlib>
#include <balance_gyro.h>
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
	Accelzero(1.616812667), Accelscale(0.206) {

	adc_init(raw_x_port);
}
void BalanceAccel::Refresh(){

//		Rx =  (((float) adc_once(raw_x_port, ADC_10bit) * Vmax/ 1023) - Accelzero)/ 0.8f;
		Rx =  (((float) adc_once(raw_x_port, ADC_10bit) * Vmax/ 1024));
//		printf("%d\n", adc_once(raw_x_port, ADC_10bit));
		Rx *= 1.179859431f;
		Rx -= -0.011406844f;

		if(Rx > 1.0){
			Rx = 1.0;
		}else if(Rx < -1.0){
			Rx = -1.0;
		}
		raw_accel_angle = 90 - (acos(Rx) * 180 / 3.1415f - 90);


}



