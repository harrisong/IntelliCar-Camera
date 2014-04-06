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

	int BlackCount = 0;
	int BlackSum = 0;
	float LineCenterX = -1;

	float LineCenterXSet[60];

	for(int i=1; i<=CAMERA_H; i++)
	{
		for(int j=1; j<=CAMERA_W; j++)
		{
			if(m_car.GetPixel(i*j-1)==BLACK)
			{
				BlackCount++;
				BlackSum+=j;
			}
		}
		LineCenterX = (int) (BlackSum / BlackCount) ;
		LineCenterXSet[i] = LineCenterX;
	}

	float s1=0; float s2=0; float s3=0; float s4=0;
	float slope;

	for(int i=0; i<60; i++)
	{
		s1+=LineCenterXSet[i]*i;
		s2+=LineCenterXSet[i];
		s3+=i;
		s4+=i*i;
	}

	slope = (60*s1 - s2*s3) / (60*s4 - s2*s2);					//from Excel equation for calculating the slope of giving n points

	float sumX=0; float sumY=0;
	float intercept;

	for(int i=0; i<60; i++)
	{
		sumX+=LineCenterXSet[i];
		sumY+=i;
	}

	intercept = (sumY/60) - (slope*sumX/60);



	LOG_W("Line equation: y = %fx + %f", slope, intercept);			//print our y = mx + c;
}

void CameraApp::SendImage(){

	m_car.UartSendChar(0);
	m_car.UartSendChar(255);
	m_car.UartSendChar(1);
	m_car.UartSendChar(0);
	//m_car.UartSendBuffer(m_car.GetCamera().GetImageBuff(), 80 * 60);

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

		TurnControl();
		BalanceControl();
		PositionControl();
		SendToMotor();


	}

}

}
