/*
 * car.h
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef CAMERA_CAR_H_
#define CAMERA_CAR_H_

#include <libsc/com/config/2014_camera.h>
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
#include <libsc/com/encoder.h>
#include "balance_gyro.h"
#include "balance_encoder.h"

#include "kalman.h"

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

	BalanceAccel* GetGyro();
	void GyroRefresh();
	float GetRawAngle();

//	BalanceEncoder* GetEncoder(int n);
//	int32 GetEncoderSpeed(int);

	libsc::Motor* GetMotor(int n);
	void MoveMotor(int, const uint16_t);
	void MotorDir(int n, const bool flag);

	libsc::Led* GetLed(int n);


	
	libsc::Lcd* GetLcd();

	Byte* ExpandPixel(const int);
	Byte* ExpandPixel(const Byte *src, const int line);
	int GetPixel(const Byte* src, const int x, const int y);
	libsc::Ov7725* GetCamera();
	const Byte* GetImage();

private:
	libsc::Led m_leds[4];
	libsc::UartDevice m_uart;
	libsc::Motor m_motor1, m_motor2;
	libsc::Ov7725 m_cam;
	libsc::Lcd m_lcd;
	BalanceAccel m_gyro;
//	BalanceEncoder m_encoder1;
//	BalanceEncoder m_encoder2;

	uint16_t m_current_time, m_prev_time, m_delta_time;

	const Byte* src;


};

}

#endif /* CAMERA_CAR_H_ */
