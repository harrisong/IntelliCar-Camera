/*
 * camera_app.cpp
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>


#include "mini_common.h"
#include "hw_common.h"
#include "camera/camera_app.h"
#include "MK60_gpio.h"

namespace camera
{
CameraApp::CameraApp():
	_gyro(0), _count(0), Speed1(0), Speed2(0),
	m_balance_pid(SETPOINT, balance_kp, balance_ki, balance_kd),
	m_speed_pid(SPEEDSETPOINT, speed_kp, speed_ki, speed_kd)
{
	libutil::Clock::Init();
	kalman_filter_init(&gyro_kf, 0.005, 0.5, 0, 1);
}

CameraApp::~CameraApp()
{

}

void CameraApp::BalanceControl()
{
	///Get values from gyro///
	m_car.GyroRefresh();

	///PD equation
	//_gyro = m_car.GetGyroOffset();
	//kalman_filtering(&gyro_kf, &_gyro, 1);
	if(m_car.GetRawAngle() < DEADZONELOWER || m_car.GetRawAngle() >  DEADZONEHIGHER) {
		Speed1 = Speed2 = 0;
	}else{
		//Speed1 = Speed2 = balance_kp * m_car.GetGyroOffset() + balance_kd * m_car.GetGyroOmega();
		Speed1 = Speed2 = m_balance_pid.Calc(m_car.GetRawAngle());
	}
	if(Speed1 > 0){
		m_dir1 = m_dir2 = false;
	}else if(Speed1 < 0){
		m_dir1 = m_dir2 = true;
	}
	//printf("Raw Angle: %d \t Speed: %d\n\r", m_car.GetRawAngle(), Speed1);
	printf("%d\r", m_car.GetGyroOffset());
	//DELAY_MS(1000);
}

void CameraApp::PositionControl(){
	///Get values from encoders///
	m_car.GetEncoder(1).refresh();
	m_car.GetEncoder(2).refresh();
	Position = (m_car.GetEncoder(1).GetTotal() + m_car.GetEncoder(2).GetTotal())/2;
	if(Position - TargetPosition > 0){

	}else if(Position  - TargetPosition < 0){

	}
}

void CameraApp::SpeedControl(){
	//m_speed_pid.Calc( );
}

void CameraApp::TurnControl(){


}

void CameraApp::SendImage(){

	/*m_car.UartSendChar(0);
	m_car.UartSendChar(255);
	m_car.UartSendChar(1);
	m_car.UartSendChar(0);
	*/

}

void CameraApp::MoveMotor(){
	m_car.MotorDir(1, m_dir1);
	m_car.MoveMotor(1,abs(Speed1));
	m_car.MotorDir(2, !m_dir2);
	m_car.MoveMotor(2,abs(Speed2));
}



void CameraApp::Run()
{

	while (true)
	{
		#if defined(DEBUG)
			//SendImage();
		#endif
		m_car.ShootOnceTest();
		//TurnControl();
		BalanceControl();
		//PositionControl();
		MoveMotor();


	}

}

}
