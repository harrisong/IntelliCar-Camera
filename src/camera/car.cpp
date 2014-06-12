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

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)}, m_uart(3, LIBSC_BT_UART_BAUD),
		  m_motor1(0), m_motor2(1),
		  m_cam(CAM_W, CAM_H),
		  m_gyro(RXADC)

		  //m_encoder1(0),
		  //m_encoder2(1)
{
	libutil::InitDefaultFwriteHandler(&m_uart);

	while (!m_cam.Init())
	{
		LOG_E("Camera fucked up!!!!!");
		DELAY_MS(250);
	}
	m_cam.ShootContinuously();

}

Car::~Car()
{
	libutil::UninitDefaultFwriteHandler();
}


BalanceAccel* Car::GetGyro(){
	return &m_gyro;
}

void Car::GyroRefresh(){
	m_gyro.Refresh();
}

float Car::GetRawAngle(){
	return m_gyro.GetRawAngle();
}

libsc::Ov7725* Car::GetCamera(){
	return &m_cam;
}
/*
BalanceEncoder* Car::GetEncoder(int n){
	switch(n){
	case 0:
		return &m_encoder1;
		break;
	case 1:
		return &m_encoder2;
		break;
	default:
		assert(0);
		break;
	}
}

int32 Car::GetEncoderSpeed(int n){
	int32 m_current_speed, m_prev_speed, m_delta_speed;
	m_current_time = libutil::Clock::Time();
	m_delta_time = m_current_time - m_prev_time;
	switch(n){
	case 0:
		m_current_speed = m_encoder1.GetCurrent();
		break;
	case 1:
		m_current_speed = m_encoder2.GetCurrent();
		break;
	default:
		assert(0);
		break;
	}
	m_delta_speed = m_current_speed - m_prev_speed;
	m_prev_speed = m_current_speed;
	m_prev_time = m_current_time;
	return m_delta_speed;
}*/

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

void Car::MoveMotor(int n, const uint16_t power){
	switch(n){
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


libsc::Led* Car::GetLed(int n)
{
	return &m_leds[n];
}

libsc::Lcd* Car::GetLcd()
{
	//return &m_lcd;
}

int Car::Clamp(const int x)
{
	if(x<0)
		return 0;
	else if(x>=CAM_W)
		return CAM_W-1;
	else
		return x;
}

Byte* Car::ExpandPixel(const Byte *src, const int line)
{
    static Byte product[CAM_W];
    Byte *it = product;
    const int offset = line * CAM_W / 8;
    for (int i = 0; i < CAM_W / 8; ++i)
    {
        *(it++) = ((src[i + offset] >> 7) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 6) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 5) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 4) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 3) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 2) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 1) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 0) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
    }
    return product;
}


int Car::GetPixel(const Byte* src, const int x, const int y)
{
    const int offset = x/8 + (y * CAM_W / 8);

    return (src[offset] >> (x%8) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
}

}
