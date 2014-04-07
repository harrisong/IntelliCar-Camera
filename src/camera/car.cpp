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
		  m_motor1(0), m_motor2(1, 0.9),
		  m_cam(CAM_W, CAM_H),
		  m_gyro(GYROADC, ANGLEADC, RZADC, RXADC, 16350),
		  m_encoder1(FTM1), m_encoder2(FTM2),
		  m_lcd(true)
{
	libutil::InitDefaultFwriteHandler(&m_uart);
	//m_uart.StartReceive();
	while (!m_cam.Init())
	{
		LOG_E("Camera fucked up!!!!!");
		DELAY_MS(250);
	}
	//camera.ShootContinuously();
	m_lcd.Clear(Lcd::GetRgb565(0x33, 0xB5, 0xE5));
}

Car::~Car()
{
	libutil::UninitDefaultFwriteHandler();
}

BalanceGyro Car::GetGyro(){
	return m_gyro;
}

void Car::GyroRefresh(){
	m_gyro.Refresh();
}

int16 Car::GetGyroOffset(){
	return m_gyro.GetOffset();
}

int16 Car::GetRawAngle(){
	return m_gyro.GetRawAngle();
}

int16 Car::GetGyroOmega(){
	return m_gyro.GetOmega();
}

BalanceEncoder Car::GetEncoder(int n){
	switch(n){
	case 1:
		return m_encoder1;
		break;
	case 2:
		return m_encoder2;
		break;
	}
}

libsc::Motor Car::GetMotor(int n){
	switch(n){
	case 1:
	default:
		return m_motor1;
		break;
	case 2:
		return m_motor2;
		break;
	}
}


libsc::Led Car::GetLed(int n)
{
	return m_leds[n];
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

void Car::ShootOnceTest(){
	libutil::Clock::ClockInt prev_time = libutil::Clock::Time();
	int frame_count = 0;
	m_cam.ShootOnce();
	//while (true)
	//{
		while (!m_cam.IsImageReady())
		{}

		const Byte *src = m_cam.LockBuffer();
		//uart.SendChar(0);
		//uart.SendChar(255);
		//uart.SendChar(1);
		//uart.SendChar(0);
		for (int i = CAM_H - 1; i >= 0; --i)
		{
			const Byte *buf = ExpandPixel(src, i);
			//uart.SendBuffer(buf, CAM_W);

			m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
			//uart.SendChar('\n');
			//delete[] buf;
		}
/*
		for(int i=0; i<120; i++)
		{
			DrawCenterPixel(src, i);
		}
*/
	   // printCenterLineEquation(LineCenterXSet);

		m_cam.UnlockBuffer();
		++frame_count;
		//uart.SendStr("\n\n\n");

		if (libutil::Clock::TimeDiff(libutil::Clock::Time(), prev_time) >= 1000)
		{
			prev_time = libutil::Clock::Time();
			printf("FPS: %d\n", frame_count);
			frame_count = 0;
		}

		//m_cam.ShootOnce();
	//}
}

}
