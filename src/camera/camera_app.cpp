/*
 * camera_app.cpp
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include "camera/camera_app.h"

namespace camera
{

CameraApp::CameraApp():
	_gyro(0), _count(0), Speed1(0), Speed2(0)
{
	kalman_filter_init(&gyro_kf, 0.005, 0.5, 0, 1);
}

CameraApp::~CameraApp()
{

}

void CameraApp::BalanceControl()
{
	///Get values from gyro///
	m_car.GetGyro().refresh();

	///PD equation
	_gyro = m_car.GetGyro().get_offset();
	kalman_filtering(&gyro_kf, &_gyro, 1);
	if(_gyro < -6600 || _gyro >  6500) {
		Speed1 = Speed2 = 0;
	}else{
		Speed1 = Speed2 = kp * _gyro + kd * m_car.GetGyro().get_omega();
	}
}

void CameraApp::PositionControl(){
	///Get values from encoders///
	m_car.GetEncoder(1).refresh();
	m_car.GetEncoder(2).refresh();
	Position = (m_car.GetEncoder(1).gettotal() + m_car.GetEncoder(2).gettotal())/2;
	if(Position - TargetPosition > 0){

	}else if(Position  - TargetPosition < 0){

	}
}

void CameraApp::TurnControl(){

	uint8_t*  ImageBuff = m_car.GetCamera().GetImageBuff();


}

void CameraApp::SendImage(){
	m_car.GetCamera().extract_to_buffer();
	m_car.UartSendChar(0);
	m_car.UartSendChar(255);
	m_car.UartSendChar(1);
	m_car.UartSendChar(0);
	m_car.UartSendBuffer((uint8_t*)m_car.GetCamera().GetImageBuff(), 80 * 60);

}

void CameraApp::SendToMotor(){
	m_car.GetMotor(1).SetPower(Speed1);
	m_car.GetMotor(2).SetPower(Speed2);
}

void CameraApp::Run()
{
	while (true)
	{
		#if defined(DEBUG)
			SendImage();
		#endif

		BalanceControl();
		PositionControl();
		SendToMotor();
	}
}

}
