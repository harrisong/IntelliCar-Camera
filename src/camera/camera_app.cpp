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
#include "libutil/string.h"

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
int abs(const int num)
{
	return num>0 ? num : -1*num;
}

int CameraApp::GetPixel(const Byte* src, const int x, const int y)
{
    const int offset = x/8 + (y * CAM_W / 8);

    return (src[offset] >> (x%8) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
}

void CameraApp::DrawCenterPixelAndPrintEquation()
{
	const Byte* src = m_car.GetImage();

	float BlackCount = 0.0;
	float BlackSum = 0.0;
	int LineCenterX;

	int LineCenterXSet[CAM_H];

	for(int y=0; y<CAM_H; y++)
	{
		for(int x=0; x<CAM_W; x++)
		{
			if(GetPixel(src, x, y) == BLACK_BYTE)
			{
				BlackCount++;
				BlackSum+=x;
			}
			LineCenterX = (int) round(BlackSum / BlackCount) ;
			LineCenterXSet[y] = LineCenterX;
		}
	}

	for(int y=0; y<CAM_H; y++)
	{										//(0,0)=(119,0), (1,0)=(119-,1), (2,3)=(116,2)
		m_car.GetLcd().DrawPixel(119-y, LineCenterXSet[y], RED_BYTE);
	}

	float s1=0; float s2=0; float s3=0; float s4=0;
	float slope;

	for(int y=0; y<CAM_H; y++)
	{
		s1+=LineCenterXSet[y]*y;
		s2+=LineCenterXSet[y];
		s3+=y;
		s4+=y*y;
	}

	slope = (CAM_H*s1 - s2*s3) / (CAM_H*s4 - s2*s2);					//from Excel equation for calculating the slope of giving n points

	float sumX=0; float sumY=0;
	float intercept;

	for(int i=0; i<CAM_H; i++)
	{
		sumX+=LineCenterXSet[i];
		sumY+=i;
	}

	intercept = (sumY/CAM_H) - (slope*sumX/CAM_H);

	m_car.GetBluetooth().SendStr(libutil::String::Format("y = %fx + %f\r\n", slope, intercept).c_str());
}

int CameraApp::GetRotationInstruction()
{
	const Byte* src = m_car.GetImage();

	int LeftBlackDot = 0;
	int RightBlackDot = 0;
	int LeftWhiteDot = 0;
	int RightWhiteDot = 0;

	for(int y=0; y<CAM_H; y++)
	{
		for(int x=0; x<CAM_W; x++)
		{
			if(GetPixel(src, x, y) == BLACK_BYTE)
			{
				if(x>=80)
					RightBlackDot++;
				else
					LeftBlackDot++;
			}
			else
			{
				if(x>=80)
					RightWhiteDot++;
				else
					LeftWhiteDot++;
			}
		}
	}

	int Difference_Black = abs(LeftBlackDot-RightBlackDot);
	int Difference_White = abs(LeftWhiteDot-RightWhiteDot);

	if(Difference_Black<=3000)
	{
		m_car.GetBluetooth().SendStr("No Turn ");
		Difference_Black=0;
	}
	else
	{
		if(LeftBlackDot>RightBlackDot)
			m_car.GetBluetooth().SendStr("Right ");
		else if(RightBlackDot>LeftBlackDot)
			m_car.GetBluetooth().SendStr("Left ");
	}

	m_car.GetBluetooth().SendStr(
		libutil::String::Format(
			"LB:%d RB:%d LW:%d RW:%d\r\n",
			LeftBlackDot, RightBlackDot, LeftWhiteDot, RightWhiteDot
		).c_str()
	);

	return (int) round((float)((LeftBlackDot - RightBlackDot))*90/19200);
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

		//DrawCenterPixelAndPrintEquation();
		int instruction = GetRotationInstruction();
		//TurnControl();
		BalanceControl();
		//PositionControl();
		MoveMotor();


	}

}

}
