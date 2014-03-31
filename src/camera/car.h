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
#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>
#include <libsc/com/motor.h>
#include "balance_gyro.h"
#include "balance_encoder.h"
#include "balance_camera.h"
#include "kalman.h"

namespace camera
{

class Car
{
public:
	Car();

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

	void balance();
	void setposition();
	void setcarspeed();
	void setservo(int16 angle);

private:
	libsc::Led m_leds[4];
	libsc::UartDevice m_uart;
	libsc::Motor motor1, motor2;
	BalanceGyro gyro;
	BalanceEncoder encoder1;
	BalanceEncoder encoder2;
	BalanceCamera camera;

	KF gyro_kf;
	float _gyro;
	int _count;

	int16 speed1, speed2;
};

}

#endif /* CAMERA_CAR_H_ */
