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
	m_gyro(0), m_balance_speed1(0), m_balance_speed2(0),
	m_speed_speed1(0), m_speed_speed2(0),
	m_balance_pid(SETPOINT, balance_kp, balance_ki, balance_kd),
	m_speed_pid(SPEEDSETPOINT, speed_kp, speed_ki, speed_kd),
	m_position_pid(POSITIONSETPOINT, position_kp, position_ki, position_kd),
	m_count(0), m_speed_control_count(0),
	m_total_speed1(0), m_total_speed2(0), m_real_total_speed(0),
	current_time(0), prev_time(0), delta_time(0),
	m_real_current_speed(0), m_real_prev_speed(0),
	prev_tempspeed(0), current_tempspeed(0)
{
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf, 1000, 1911.92083f, 0, 1);
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
	//if(m_count%100 == 99) printf("%f, %f\n\r", m_car.GetRawAngle(), m_car.GetGyroOmega());
	///PD equation
	m_gyro = (float) m_car.GetRawAngle();
	kalman_filtering(&m_gyro_kf, &m_gyro, 1);
	if(m_car.GetRawAngle() < DEADZONELOWER || m_car.GetRawAngle() >  DEADZONEHIGHER) {
		m_balance_speed1 = m_balance_speed2 = 0;
	}else{
		//Speed1 = Speed2 = balance_kp * m_car.GetGyroOffset() + balance_kd * m_car.GetGyroOmega();
		m_balance_speed1 = m_balance_speed2 = m_balance_pid.Calc( (int16_t) m_gyro );
	}

	//printf("Angle:%d, Speed:%d\n\r", (int16_t) m_gyro, m_balance_speed1);


	//m_count++;

}

void CameraApp::PositionControl(){
	///Get values from encoders///
	encoder1 = FTM_QUAD_get(FTM1);
	encoder2 = -FTM_QUAD_get(FTM2);
	printf("Encoder: %d\n\r", (encoder2+encoder1)/2);
	int16_t tempspeed = m_position_pid.Calc((encoder2+encoder1)/2);
	//m_total_speed1 += tempspeed;
	//m_total_speed2 += tempspeed;
}

void CameraApp::SpeedControl(){
	current_time = libutil::Clock::Time();
	delta_time = current_time - prev_time;
	static int32_t error, total_error;
	if(m_count%100==99){
		prev_time = current_time;
		m_real_current_speed = (FTM_QUAD_get(FTM1) + (-FTM_QUAD_get(FTM2)))/2;
		//m_real_current_speed = - FTM_QUAD_get(FTM2);
		//int16_t m_real_delta_speed = m_real_current_speed;
		FTM_QUAD_clean(FTM1);
		FTM_QUAD_clean(FTM2);
		//m_real_prev_speed = m_real_current_speed;
		error = SPEEDSETPOINT - m_real_current_speed;
		total_error += error;

		prev_tempspeed = current_tempspeed;
		current_tempspeed = speed_kp * error + speed_ki * total_error;
	}

	//printf("Encoder Feedback: %d \t P: %d\n\r",  m_real_current_speed, tempspeed);

	//tempspeed = m_speed_pid.Calc(m_real_current_speed);


		m_speed_speed1 = m_speed_speed2 = (current_tempspeed - prev_tempspeed) * (++m_speed_control_count) / 100 + prev_tempspeed;
		if(m_speed_control_count==100) m_speed_control_count = 0;

	//printf("Total Speed: %d \t Output Speed: %d \t Encoder Feedback: %d\n\r", m_total_speed1, -tempspeed, m_real_current_speed);
	//DELAY_MS(5);

}

void CameraApp::TurnControl(){


}

void CameraApp::MoveMotor(){
		//printf("Total Speed: %d \t Balance Output Speed: %d \t Encoder Output Speed: %d \t Encoder Feedback: %d\n\r", m_total_speed1, m_balance_speed1, m_speed_speed1, m_real_current_speed);
		m_total_speed1 = m_balance_speed1 - m_speed_speed1;
		m_total_speed2 = m_balance_speed2 - m_speed_speed2;
		m_total_speed1 = m_total_speed2 = 0;


		if(m_total_speed1 > 0){
			m_dir1 = m_dir2 = true;
		}else if(m_total_speed1 < 0){
			m_dir1 = m_dir2 = false;
		}

		m_car.MotorDir(0, m_dir1); 			////Right Motor - True Backward  -  False Forward
		m_car.MoveMotor(0,(uint16_t) abs(m_total_speed1));

		m_car.MotorDir(1, !m_dir2);			////Left Motor - False Backward  -  True Forward
	    m_car.MoveMotor(1,(uint16_t) abs(m_total_speed2));

}



void CameraApp::Run()
{
	printf("test");
	adc_init(ADC1_SE4a);

	gpio_init(PTD15, GPI, 1);

	while(gpio_get(PTD15)==1);

	uint16_t p_time=0, c_time=0;
	float kp = balance_kp;
	while (true)
	{
		c_time = libutil::Clock::Time();
		if(libutil::Clock::TimeDiff(c_time,p_time)>=1){
			//m_total_speed1 = m_total_speed2 = 0;
			p_time = c_time;
			switch(m_count%2){
			case 0:
				//m_car.ShootOnceTest();
				break;
			case 1:
			{
				//n = adc_once(ADC1_SE4a, ADC_16bit);
				//n2 = n*(DEADZONEHIGHER - DEADZONELOWER)/65535;
				//m_car.GetGyro()->ChangeSetPoint(n2+DEADZONELOWER);
				//m_balance_pid.SetKd(n2);

				while(gpio_get(PTD15)==0){
				//n = adc_once(ADC1_SE4a, ADC_16bit);
				//m_balance_pid.SetSetpoint((int16)n*(DEADZONEHIGHER-DEADZONELOWER)/65535+DEADZONELOWER);
				//printf("SP:%d\n\r", n*(DEADZONEHIGHER-DEADZONELOWER)/65535+DEADZONELOWER);
				}
				/*char ch;
				if(m_car.GetBluetooth()->PeekChar(&ch)){
					if(ch=='1'){
						kp += 0.01;
						m_balance_pid.SetKp(kp);
					}else if(ch=='2'){
						kp -= 0.01;
						m_balance_pid.SetKp(kp);
					}
					printf("Kp:%f\n\r",kp);
				}*/

				BalanceControl();
				//printf("Total Speed: %d \t Balance Output Speed: %d \t Encoder Output Speed: %d \t Encoder Feedback: %d\n\r", m_total_speed1, m_balance_speed1, m_speed_speed1, m_real_current_speed);
				//SpeedControl();

				break;
			}

			default:
				break;
			}
			//if(m_count%4==3)
				//SpeedControl();
			m_count++;

			//DrawCenterPixelAndPrintEquation();
			//int instruction = GetRotationInstruction();
			//TurnControl();
			//PositionControl();
			MoveMotor();
		}

	}

}

}
