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
#include <libutil/tunable_int_manager.h>
#include <libsc/com/ov7725.h>
#include <libsc/com/lcd.h>
#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>
#include <libsc/com/motor.h>
#include <libsc/com/encoder.h>
#include "balance_gyro.h"
#include "balance_encoder.h"
#include <libsc/com/joystick.h>
#include <libsc/com/button.h>
#include "kalman.h"

namespace camera
{

class Car
{
public:
	Car();
	~Car();

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

	libsc::UartDevice* GetUart()
	{
		return &m_uart;
	}

	BalanceAccel* GetGyro();
	libsc::Motor* GetMotor(int n);
	libsc::Led* GetLed(int n);
	libsc::Lcd* GetLcd();
	libsc::Ov7725* GetCamera();
	libsc::Button* GetButton();
	libsc::Joystick* GetJoystick();

	void GyroRefresh();
	float GetRawAngle();

	void MoveMotor(int, const uint16_t);
	void MotorDir(int n, const bool flag);


private:
	libsc::Led m_leds[4];
	libsc::UartDevice m_uart;
	libsc::Motor m_motor1, m_motor2;
	libsc::Ov7725 m_cam;
	libsc::Lcd m_lcd;
	BalanceAccel m_gyro;
	libsc::Button m_start_button;
	libsc::Joystick m_joystick;
	libutil::TunableInt *m_kp;

	uint16_t m_current_time, m_prev_time, m_delta_time;

};

}

#endif /* CAMERA_CAR_H_ */
