/*
 * car.cpp
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include "camera/car.h"

using namespace libsc;

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)}, m_uart(3, 115200),
		  motor1(0), motor2(1, 0.9),
		  gyro(GYROADC, ANGLEADC, RZADC, RXADC, 12200),
		  encoder1(FTM1), encoder2(FTM2),
		  _gyro(0), _count(0)
{
	kalman_filter_init(&gyro_kf, 0.005, 0.5, 0, 1);
	m_uart.StartReceive();
}

void Car::balance(){
	///Get values from encoder, gyro///
	encoder1.refresh();
	encoder2.refresh();
	gyro.refresh();

	///PD equation
	_gyro = gyro.get_offset();
	kalman_filtering(&gyro_kf, &_gyro, 1);
	if(gyro.get_offset() < -6600 || gyro.get_offset() >  6500) {
		speed1 = speed2 = 0;
		motor1.SetPower(0);
	}else{
		speed1 = speed2 = kp * _gyro + kd * gyro.get_omega();
	}
	motor1.SetPower(abs(speed1));
	motor2.SetPower(abs(speed2));

}

}
