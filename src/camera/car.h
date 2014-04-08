/*
 * car.h
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef CAMERA_CAR_H_
#define CAMERA_CAR_H_

#include <vars.h>
#include <cstdint>
#include <cstdlib>
#include <vectors.h>
#include <libutil/clock.h>
#include <libsc/com/ov7725.h>
#include <libsc/com/lcd.h>
#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>
#include <libsc/com/motor.h>
#include "balance_gyro.h"
#include "balance_encoder.h"

#include "kalman.h"

#define CAM_W 160
#define CAM_H 120
#define WHITE_BYTE 0xFF
#define BLACK_BYTE 0						//online RGB 565 calculator
#define RED_BYTE 0xF800						//http://www.henningkarlsen.com/electronics/calc_rgb565.php

namespace camera
{

class Car
{
public:
	Car();
	~Car();

	/**
	 * Switch on/off the LEDs
	 *
	 * @param obj
	 * @param id The id of the LED, [0, 3]
	 * @param flag
	 */
	void SwitchLed(const uint8_t id, const bool flag)
	{
		m_leds[id].SetEnable(flag);
	}

	void UartSendChar(const char c)
		{
			m_uart.SendChar(c);
		}

	void UartSendStr(const char *str)
	{
		m_uart.SendStr(str);
	}

	void UartSendBuffer(const uint8_t *buf, const uint32_t len)
	{
		m_uart.SendBuffer(buf, len);
	}

	bool UartPeekChar(char *out_ch)
	{
		return m_uart.PeekChar(out_ch);
	}
	BalanceGyro GetGyro();
	void GyroRefresh();
	int16 GetGyroOffset();
	int16 GetGyroOmega();
	int16 GetRawAngle();
	BalanceEncoder GetEncoder(int n);
	libsc::Motor GetMotor(int n);
	libsc::Led GetLed(int n);
	Byte* ExpandPixel(const Byte *, const int);
	void ShootOnceTest();
	void ShootContinuouslyTest();
	void DrawCenterPixel(const Byte* src, const int line);
	int GetPixel(const int x, const int y, const int offset);
	void PrintCenterLineEquation(float LineCenterX[]);

	void MoveMotor(int, const uint16_t);

private:
	libsc::Led m_leds[4];
	libsc::UartDevice m_uart;
	libsc::Motor m_motor1, m_motor2;
	libsc::Ov7725 m_cam;
	libsc::Lcd m_lcd;
	BalanceGyro m_gyro;
	BalanceEncoder m_encoder1;
	BalanceEncoder m_encoder2;



};

}

#endif /* CAMERA_CAR_H_ */
