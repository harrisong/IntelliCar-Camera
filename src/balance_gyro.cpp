/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
//#include <cstdlib>
#include <camera/camera_app.h>
#include <balance_gyro.h>

#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>
//#include <FIRE_MK60_conf.h>
//#include <float.h>

//#include <mini_common.h>
//#include <hw_common.h>
//#include <MK60_pit.h>
//#include <MK60_gpio.h>
//#include <MK60_i2c.h>
using namespace camera;
using namespace libutil;
//extern CameraApp* cameraApp;

BalanceAccel::BalanceAccel(ADCn_Ch_e p3):
	raw_x_port(p3),
	raw_accel_angle(90),
	Rx(0),
	Vmax(3.3), Adc16max(65535),
	Accelzero(1.616812667), Accelscale(0.206)
{

	adc_init(raw_x_port);
}



void BalanceAccel::Refresh(){

//		Rx =  (((float) adc_once(raw_x_port, ADC_10bit) * Vmax/ 1023) - Accelzero)/ 0.8f;
		Rx =  (((float) adc_once(raw_x_port, ADC_10bit) * Vmax / 1024.0f)) - 1.65f;

		Rx *= 1.3f;
		Rx -= -0.018f;
//		Rx -= TunableInt::AsFloat((app->tunableints[13])->GetValue());

		if(Rx > 1.0){
			Rx = 1.0;
		}else if(Rx < -1.0){
			Rx = -1.0;
		}
//		raw_accel_angle = Rx * 180 / 3.1415f + 90 - setpoint;
		raw_accel_angle = asin(Rx) * 57.2957795f;

//		printf("%g\n", raw_accel_angle);
}



