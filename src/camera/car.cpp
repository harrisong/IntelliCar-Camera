/*
 * car.cpp
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <hw_common.h>
#include <log.h>
#include "camera/car.h"
#include <libutil/misc.h>

using namespace libsc;

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)}, m_uart(3, 115200),
		  motor1(0), motor2(1, 0.9),
		  camera(CAMERA_H, CAMERA_W),
		  gyro(GYROADC, ANGLEADC, RZADC, RXADC, 12200),
		  encoder1(FTM1), encoder2(FTM2)
{
	libutil::InitDefaultFwriteHandler(&m_uart);
	//m_uart.StartReceive();
	while (!camera.Init())
	{
		LOG_E("Camera fucked up!!!!!");
		DELAY_MS(250);
	}
	camera.ShootContinuously();
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
COLORS Car::GetImgBuff(int n){
	GetCamera();
	return img_buff[n];
}

void Car::GetCamera(){
	const Byte* img = camera.LockBuffer();
	COLORS colour[2]={WHITE,BLACK};
	uint8_t tmpsrc;

	int srclen = camera.GetImageW() * camera.GetImageH();
	while(srclen --)
	{
		tmpsrc = *img++;
		*img_buff++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
		*img_buff++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}

	camera.UnlockBuffer();
}

}
