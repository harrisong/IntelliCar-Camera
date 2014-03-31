/*
 * car.cpp
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include "camera/car.h"
#include <libutil/misc.h>

using namespace libsc;

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)}, m_uart(3, 115200),
		  motor1(0), motor2(1, 0.9),
		  gyro(GYROADC, ANGLEADC, RZADC, RXADC, 12200),
		  encoder1(FTM1), encoder2(FTM2)
{
	libutil::InitDefaultFwriteHandler(&m_uart);
	//m_uart.StartReceive();
}

Car::~Car()
{
	libutil::UninitDefaultFwriteHandler();
}

BalanceGyro Car::GetGyro(){
	return gyro;
}

BalanceEncoder Car::GetEncoder(int n){
	switch(n){
	case 1:
		return encoder1;
		break;
	case 2:
		return encoder2;
		break;
	}
}

BalanceCamera Car::GetCamera(){
	return camera;
}

libsc::Motor Car::GetMotor(int n){
	switch(n){
	case 1:
	default:
		return motor1;
		break;
	case 2:
		return motor2;
		break;
	}
}

}
