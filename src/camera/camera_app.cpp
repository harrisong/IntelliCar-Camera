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
#include <MK60_gpio.h>
#include <MK60_adc.h>
#include "libutil/string.h"

namespace camera
{
CameraApp::CameraApp():
	m_gyro(0), m_speed1(0), m_speed2(0),
	m_balance_pid(SETPOINT, balance_kp, balance_ki, balance_kd),
	m_speed_pid(SPEEDSETPOINT, speed_kp, speed_ki, speed_kd),
	m_count(0)
{
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf, 0.005, 0.5, 0, 1);
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
		m_car.GetLcd()->DrawPixel(119-y, LineCenterXSet[y], RED_BYTE);
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

	m_car.GetBluetooth()->SendStr(libutil::String::Format("y = %fx + %f\r\n", slope, intercept).c_str());
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
		m_car.GetBluetooth()->SendStr("No Turn ");
		Difference_Black=0;
	}
	else
	{
		if(LeftBlackDot>RightBlackDot)
			m_car.GetBluetooth()->SendStr("Right ");
		else if(RightBlackDot>LeftBlackDot)
			m_car.GetBluetooth()->SendStr("Left ");
	}

	m_car.GetBluetooth()->SendStr(
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
	m_gyro = (float) m_car.GetRawAngle();
	kalman_filtering(&m_gyro_kf, &m_gyro, 1);
	if(m_car.GetRawAngle() < DEADZONELOWER || m_car.GetRawAngle() >  DEADZONEHIGHER) {
		m_speed1 = m_speed2 = 0;
	}else{
		//Speed1 = Speed2 = balance_kp * m_car.GetGyroOffset() + balance_kd * m_car.GetGyroOmega();
		m_speed1 = m_speed2 = m_balance_pid.Calc( (int16_t) m_gyro );
	}

	if(m_speed1 > 0){
		m_dir1 = m_dir2 = true;
	}else if(m_speed1 < 0){
		m_dir1 = m_dir2 = false;
	}
	printf("Angle: %d \t Speed2: %d \n\r", m_car.GetRawAngle(), m_speed1);
	//m_count++;
	//DELAY_MS(1000);
}

void CameraApp::PositionControl(){
	///Get values from encoders///
	//m_car.GetEncoder(1)->refresh();
	m_car.GetEncoder(1)->refresh();
	//m_position = (m_car.GetEncoder(1)->GetTotal() + m_car.GetEncoder(2)->GetTotal())/2;
	m_position = m_car.GetEncoder(1)->GetTotal();

	if(m_position - m_target_position > 0){

	}else if(m_position  - m_target_position < 0){

	}
}

void CameraApp::SpeedControl(){
	m_count++;
	m_car.GetEncoder(1)->refresh();
	m_encoder_speed2 =  m_car.GetEncoderSpeed(1);
	//if(m_count % 5 == 0) printf("Speed: %d\r\n", m_encoder_speed2);

	//m_speed_pid.Calc( );
}

void CameraApp::TurnControl(){


}

void CameraApp::MoveMotor(){
		m_car.MotorDir(0, !m_dir1); 			////Right Motor - True Backward  -  False Forward
		m_car.MoveMotor(0,(uint16_t) abs(m_speed1));
		//m_car.MoveMotor(0,5000);

		m_car.MotorDir(1, m_dir2);			////Left Motor - False Backward  -  True Forward
	    m_car.MoveMotor(1,(uint16_t) abs(m_speed2));
		//m_car.MoveMotor(1,5000);

}



void CameraApp::Run()
{
	int count=0;
	int button_count = 0;
	/*adc_init(ADC1_SE4a);
	n = adc_once(ADC1_SE4a, ADC_16bit);
	gpio_init(PTD15, GPI, 1);

    while(gpio_get(PTD15)==1)
    {m_car.GyroRefresh();
	m_gyro = (float) m_car.GetRawAngle();
	kalman_filtering(&m_gyro_kf, &m_gyro, 1);
	//m_car.GetGyro()->ChangeSetPoint((uint16)m_gyro);
	m_balance_pid.SetSetpoint((int32)m_gyro);
	printf("SetPoint: %d", (uint16) m_gyro);
    }*/
	while (true)
	{
		switch(count%2){
		case 0:
			m_car.ShootOnceTest();
			break;
		case 1:
			//n = adc_once(ADC1_SE4a, ADC_16bit);
			//n2 = n*(DEADZONEHIGHER - DEADZONELOWER)/65535;
			//m_car.GetGyro()->ChangeSetPoint(n2+DEADZONELOWER);
			//m_balance_pid.SetKd(n2);

			BalanceControl();

			break;
		default:
			break;
		}
		count++;

		//DrawCenterPixelAndPrintEquation();
		//int instruction = GetRotationInstruction();
		//TurnControl();
		//PositionControl();
		MoveMotor();


	}

}

}
