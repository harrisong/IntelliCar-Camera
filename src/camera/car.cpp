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
#include <assert.h>


using namespace libsc;
using namespace libutil;

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)},
		  m_uart(3, 115200),
		  m_motor1(0), m_motor2(1),
#ifdef LIBSC_USE_CAMERA
		  m_cam(CAM_W, CAM_H),
#endif
		  m_accel(RXADC),
		  m_lcd(true),
		  m_start_button(0),
		  m_joystick(0),
		  m_balancevolt(LIBSC_VOLT)
{
#ifdef LIBSC_USE_UART
	m_uart.StartReceive();
	libutil::InitDefaultFwriteHandler(&m_uart);
#endif

#ifdef LIBSC_USE_CAMERA
	while (!m_cam.Init())
	{
		LOG_E("Camera fucked up!!!!!");
		DELAY_MS(250);
	}
	m_cam.ShootContinuously();
#endif

#ifdef LIBSC_USE_K60_ENCODERS
	FTM_QUAD_Init(FTM1);
	FTM_QUAD_Init(FTM2);
#endif
}

Car::~Car()
{
	libutil::UninitDefaultFwriteHandler();
}


BalanceAccel* Car::GetAccel(){
	return &m_accel;
}

void Car::AccelRefresh(){
	m_accel.Refresh();
}

float Car::GetRawAngle(){
	return m_accel.GetRawAngle();
}

libsc::Ov7725* Car::GetCamera(){
#ifdef LIBSC_USE_CAMERA
	return &m_cam;
#endif
}

libsc::Lcd* Car::GetLcd()
{
	return &m_lcd;
}

libsc::Motor* Car::GetMotor(int n){
	switch(n){
	case 0:
		return &m_motor1;
		break;
	case 1:
		return &m_motor2;
		break;
	default:
		assert(0);
		break;
	}
}

libsc::Joystick* Car::GetJoystick()
{
	return & m_joystick;
}

libsc::Button* Car::GetButton()
{
	return &m_start_button;
}

void Car::MoveMotor(int n, const uint16_t power){
	switch(n)
	{
		case 0:
			m_motor1.SetPower(power);
			break;
		case 1:
			m_motor2.SetPower(power);
			break;
	}
}

void Car::MotorDir(int n,const bool flag){
	switch(n){
		case 0:
			m_motor1.SetClockwise(flag);
			break;
		case 1:
			m_motor2.SetClockwise(flag);
			break;
	}

}

}
