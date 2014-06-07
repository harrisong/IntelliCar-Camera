/*
 * camera_app.cpp
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>
#include "vars.h"
#include <libsc/com/config/2014_camera.h>
#include "mini_common.h"
#include "hw_common.h"
#include <cstring>
#include "camera/camera_app.h"
#include <MK60_gpio.h>
#include <MK60_adc.h>
#include <MK60_i2c.h>
#include <libutil/string.h>
#include "mpu6050.h"
#include <libsc/com/lcd.h>
#include <libsc/com/joystick.h>
#include <libsc/com/button.h>

namespace camera
{
int16_t SPEEDSETPOINT = 0;

int16_t LeftEdgeX[CAM_H];
int16_t RightEdgeX[CAM_H];
int16_t CenterX[CAM_H];

CameraApp::CameraApp():
	m_gyro(0), m_balance_speed1(0), m_balance_speed2(0),
	m_control_speed1(0), m_control_speed2(0),
	m_balance_pid(SETPOINT, balance_kp, balance_ki, balance_kd),
	m_speed_pid(SPEEDSETPOINT, speed_kp, speed_ki, speed_kd),
	m_count(0),
	m_total_speed1(0), m_total_speed2(0),
	prev_tempspeed(0), current_tempspeed(0),
	m_lcd(true),
	m_turn_speed1(0), m_turn_speed2(0)
{
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf[0], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[1], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[2], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_acc_kf, 0.0005, 0.05, 0, 1);

	memset(CenterX, -1, CAM_H);
	memset(LeftEdgeX, -1, CAM_H);
	memset(RightEdgeX, -1, CAM_H);
	m_lcd.Clear(0xFFFF);

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


void CameraApp::BalanceControl()
{
	///Get values from gyro///
	m_car.GyroRefresh();

	static float acc = m_car.GetRawAngle();
	kalman_filtering(&m_acc_kf, &acc, 1);
	m_gyro = 1/TIMECONST * acc + (1- 1/TIMECONST) * angle[0];
	static float e, delta, o_error;

	e = SETPOINT - m_gyro;
	delta = e - o_error;
	o_error = e;


	m_balance_speed1 = m_balance_speed2 = balance_kp * e + balance_kd * delta;
//	m_balance_speed1 = m_balance_speed2 = m_balance_pid.Calc( (int16_t) m_gyro );


}


void CameraApp::SpeedControl(){
	static int32_t error, total_error, delta_error, o_error = 0;

	m_encoder_2 = -(FTM_QUAD_get(FTM1)+FTM_QUAD_get(FTM2))/2;
	FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);

	error = SPEEDSETPOINT - m_encoder_2;
	delta_error = error - o_error;
	total_error += error;
	o_error = error;

	prev_tempspeed = current_tempspeed;
	current_tempspeed = speed_kp * error + speed_kd * delta_error + speed_ki * total_error;
}

void CameraApp::SpeedControlOutput(){
    static int32_t speed_control_count;

    m_control_speed1 = m_control_speed2 = (current_tempspeed - prev_tempspeed) * (++speed_control_count) / SPEEDCONTROLPERIOD + prev_tempspeed;
    if(speed_control_count==SPEEDCONTROLPERIOD) speed_control_count = 0;
}

Byte* ExpandPixel(const Byte *src, const int line)
{
    static Byte product[CAM_W];
    Byte *it = product;
    const int offset = line * CAM_W / 8;
    for (int i = 0; i < CAM_W / 8; ++i)
    {
        *(it++) = ((src[i + offset] >> 7) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 6) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 5) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 4) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 3) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 2) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 1) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 0) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
    }
    return product;
}


void CameraApp::EdgeDetection(const Byte* src, const int y)
{
	LeftEdgeX[y] = -1;
	RightEdgeX[y] = -1;

	if(y-2>=0)
		CenterX[y] = CenterX[y-1] + CenterX[y-1] - CenterX[y-2];
	else
		CenterX[y] = CAM_W/2;

	for(int x=CenterX[y]; x>=0; x--)
	{
		if(GetPixel(src, x, y) != WHITE_BYTE && x-1>=0 && x-2>=0 && GetPixel(src, x-1, y) != WHITE_BYTE && GetPixel(src, x-2, y) != WHITE_BYTE)
		{
			LeftEdgeX[y] = x+1;
		}
	}

	for(int x=CenterX[y]; x<CAM_W; x++)
	{
		if(x+1<CAM_W && x-2<CAM_W && GetPixel(src, x, y) != WHITE_BYTE && GetPixel(src, x+1, y) != WHITE_BYTE && GetPixel(src, x+2, y) != WHITE_BYTE)
		{
			RightEdgeX[y] = x-1;
		}
	}

	if(LeftEdgeX[y]==-1 || RightEdgeX[y]==-1)
	{
		if(LeftEdgeX[y]==-1)
			LeftEdgeX[y] = 0;
		if(RightEdgeX[y]==-1)
			RightEdgeX[y] = CAM_W-1;

		CenterX[y] = (int) round((((int) round( (LeftEdgeX[y] + RightEdgeX[y]) / 2)) + CenterX[y])/2);
	}
	else
	{
		CenterX[y] = (int) round((LeftEdgeX[y] + RightEdgeX[y])/2);
	}

}

void CameraApp::TurnControl(){

	static int areaPrevError = 0;
	static int encoderPrevError = 0;
	const Byte* src = m_car.GetCamera()->LockBuffer();


	int LeftWhiteDot = 0;
	int RightWhiteDot = 0;

	for(int y=0; y<CAM_H; y++)
	{
		for(int x=0; x<CAM_W; x++)
		{
			if(GetPixel(src, x, y) == WHITE_BYTE)
			{
				x>=(CAM_W/2) ? RightWhiteDot++ : LeftWhiteDot++;
			}
		}
	}

	int areaCurrentError = RightWhiteDot - LeftWhiteDot;			//http://notepad.cc/smartcar

//	double encoderCurrentError = m_encoder_2 * 2;

	int degree = (int) round(degree_kp * areaCurrentError + degree_kd * -omega[1]);
//	int degree = (int) round((degree_kp * areaCurrentError + degree_kd * ((areaPrevError - areaCurrentError) /*+ (encoderCurrentError - encoderPrevError)*/)));

	areaPrevError = areaCurrentError;
	//encoderPrevError = encoderCurrentError;

	if(degree < 0)
		degree = degree < -100 ? -100 : degree;
	else
		degree = degree > 100 ? 100 : degree;

	m_turn_speed1 = - 1 * degree * 17;
	m_turn_speed2 = 1 * degree * 17;

	m_car.GetCamera()->UnlockBuffer();
	//DELAY_MS(10);
}

void CameraApp::PrintCam(){

	const Byte* src = m_car.GetCamera()->LockBuffer();

	for (int i = CAM_H - 1; i >= 0; --i)
	{
		const Byte *buf = ExpandPixel(src, i);
		m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
	}


	for(int y=0; y<CAM_H; y++)
	{
		EdgeDetection(src, y);

		if(LeftEdgeX[y]!=-1)
			m_lcd.DrawPixel(LeftEdgeX[y], y, RED_BYTE);
		if(RightEdgeX[y]!=-1)
			m_lcd.DrawPixel(RightEdgeX[y], y, RED_BYTE);
		if(CenterX[y]!=-1)
			m_lcd.DrawPixel(RightEdgeX[y], y, RED_BYTE);
	}



	m_car.GetCamera()->UnlockBuffer();
	uint16_t sec = libutil::Clock::Time()/1000;
	const char* s = libutil::String::Format("Time: %02d",sec).c_str();
	Printline(m_lcd.FONT_H * 7, s);
//		DELAY_MS(10);
}

void CameraApp::TurnControlOutput(){

}

void CameraApp::MoveMotor(){
	static uint16_t t = 0;
	static uint32_t sum = 0;
	m_total_speed1 = m_balance_speed1 + m_control_speed1 + m_turn_speed1;
	m_total_speed2 = m_balance_speed2 + m_control_speed2 + m_turn_speed2;
	sum+=(m_total_speed1 + m_total_speed2)/2;
	m_total_speed1 = m_total_speed1 * 90 / 100;

	/*if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t) >= 50){
		t = libutil::Clock::Time();
		if(sum >= 50*3000){
			m_total_speed1 = m_total_speed2 = 0;
		}
		sum = 0;
	}*/

	if(m_total_speed1 > 0){
		m_dir1 = m_dir2 = false;
	}else if(m_total_speed1 < 0){
		m_dir1 = m_dir2 = true;
	}

	m_car.MotorDir(0, !m_dir1); 			////Right Motor - True Backward  -  False Forward
	m_car.MoveMotor(0,(uint16_t) abs(m_total_speed1));

	m_car.MotorDir(1, !m_dir2);			////Left Motor - False Backward  -  True Forward
	m_car.MoveMotor(1,(uint16_t) abs(m_total_speed2));
}



void CameraApp::Printline(uint8_t y, const char* s){
	uint8_t x=m_lcd.FONT_W;
	if(y==0){
		for(int i=0; i<x; i++) m_lcd.DrawChar(i, y, ' ', 0xFFFF, 0);
	}
	while(*s){
		if(y==0) m_lcd.DrawChar(x, y, *s, 0xFFFF, 0);
		else m_lcd.DrawChar(x, y, *s, 0, 0xFFFF);
		x+=m_lcd.FONT_W;
		s++;
	}
	if(y==0){
		for(int i=x; i<m_lcd.W; i++) m_lcd.DrawChar(i, y, ' ', 0xFFFF, 0);
	}
}

void CameraApp::PrintPtr(uint8_t y){
	for(int i=m_lcd.FONT_H+1; i<m_lcd.H; i++) m_lcd.DrawChar(0, i, ' ', 0xFFFF, 0xFFFF);
	m_lcd.DrawChar(0, y, '>', 0, 0xFFFF);
}


void CameraApp::Run()
{
	libsc::Button m_start_but(0);
	libsc::Joystick m_joystick(0);


	int maxh = m_lcd.H/m_lcd.FONT_H - 1;
	int maxw = m_lcd.W/m_lcd.FONT_W - 1;




	mpu6050_init();

	m_lcd.Clear(0xFFFF);


	int mode = -1;
	int ptr_pos = 1;
	int ptr_output_pos = 1;
	int maxchoices = 11;
	int viewport[2] = {1, 9};


	const char* s[maxchoices];
	s[0] = "Choose Mode:";
	s[1] = "Auto";
	s[2] = "PID";
	s[3] = "Accel & Gyro";
	s[4] = "Speed";
	s[5] = "Camera";
	s[6] = "Parade";
	s[7] = "Balance Only";
	s[8] = "Camera Move";
	s[9] = "Balance & Speed";
	s[10] = "Move motor";
	s[11] = "UART";

	for(int i=0; i<=maxh; i++) Printline(m_lcd.FONT_H * i, s[i]);

	PrintPtr(ptr_output_pos * m_lcd.FONT_H);

	while(mode==-1){
		switch (m_joystick.GetState())
		{
		case libsc::Joystick::DOWN:

			if(ptr_pos + 1 <= maxchoices){
				++ptr_pos;
				++ptr_output_pos;
				ptr_output_pos = ptr_output_pos > maxh ? maxh : ptr_output_pos;

				if(ptr_pos > viewport[1]){
					viewport[0]++;
					viewport[1]++;
					for(int i=1; i<=maxchoices; i++) Printline(m_lcd.FONT_H * i, "                ");
					for(int i=1; i<=maxchoices; i++) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
				}

				PrintPtr(ptr_output_pos * m_lcd.FONT_H);
			}

			DELAY_MS(150);
			break;

		case libsc::Joystick::UP:
			if(ptr_pos - 1 >= 1){
				--ptr_pos;
				--ptr_output_pos;
				ptr_output_pos = ptr_output_pos < 1 ? 1 : ptr_output_pos;

				if(ptr_pos < viewport[0]){
					viewport[0]--;
					viewport[1]--;
					for(int i=1; i<=maxchoices; i++) Printline(m_lcd.FONT_H * i, "                ");
					for(int i=1; i<=maxchoices; i++) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
				}

				PrintPtr(ptr_output_pos * m_lcd.FONT_H);
			}

			DELAY_MS(150);

			break;

		case libsc::Joystick::SELECT:
			mode = ptr_pos;
			break;
		}

	}

	uint16_t t = 0;
	uint16_t nt = 0;
	uint16_t pt = 0;

	bool autoprint = false;
	bool speedInit = false;
	m_lcd.Clear(0xFFFF);
	switch(mode){
	case 1:
		Printline(m_lcd.FONT_H * 0, "AUTO Mode");

		///////////////////////////AUTO///////////////////////////

		while(m_start_but.IsUp());

		pt = libutil::Clock::Time();

		while (true)
		{
			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				if(t - pt> 5000){
					if(!speedInit)
					{
						SPEEDSETPOINT = -60;
						speedInit = true;
					}
					else if(t%1000 == 0)
					{
						if(SPEEDSETPOINT-5 >= -100)
						{
							SPEEDSETPOINT -= 5;
						}
					}
				}


	//			if(t%150==0 && autoprint) {
	//				const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
	//				Printline(m_lcd.FONT_H * 1, s);
	//				s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
	//				Printline(m_lcd.FONT_H * 2, s);
	//				s = libutil::String::Format("Angle: %d",(int)m_gyro).c_str();
	//				Printline(m_lcd.FONT_H * 3, s);
	//				s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
	//				Printline(m_lcd.FONT_H * 4, s);
	//			}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				//TurnControlOutput();


				if(t%2==0){

					mpu6050_update();
				}

				if(t%5==0){

					BalanceControl();
				}



				if(t%45==0){ TurnControl(); }

				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0){ SpeedControl(); }
				MoveMotor();

				m_count++;
			}


			if(m_start_but.IsDown()) {
				autoprint = !autoprint;
				while(m_start_but.IsDown());
			}

		}
		///////////////////////////AUTO///////////////////////////

		break;
	case 2:
		break;
	case 3:
		Printline(m_lcd.FONT_H * 0, "Accel & Gyro");
		///////////////////////////Accel & Gyro///////////////////////////
		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				///Update Gyro every 2ms///
				if(t % 2 == 0)	{
					mpu6050_update();

				}


				if(t%150==0) {
					BalanceControl();
					const char* s = libutil::String::Format("Angle: %03d",(int)m_gyro).c_str();
					Printline(m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("Accel: %03d",(int)m_car.GetRawAngle()).c_str();
					Printline(m_lcd.FONT_H * 2, s);

					/*s = libutil::String::Format("Gyro: %03d", (int)angle[0]).c_str();
					Printline(m_lcd.FONT_H * 3, s);*/
					s = libutil::String::Format("A0: %03d", (int)angle[0]).c_str();
					Printline(m_lcd.FONT_H * 4, s);
					s = libutil::String::Format("A1: %03d", (int)angle[1]).c_str();
					Printline(m_lcd.FONT_H * 5, s);
					s = libutil::String::Format("A2: %03d", (int)angle[2]).c_str();
					Printline(m_lcd.FONT_H * 6, s);


				}


				m_count++;
			}

		}
		///////////////////////////Gyro///////////////////////////
		break;
	case 4:
		///////////////////////////Speed///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Speed Mode");
		const char* s;
		FTM_QUAD_Init(FTM1);
		FTM_QUAD_Init(FTM2);
		int16 encoder1, encoder2;
		while (true)
		{
			encoder1 = -FTM_QUAD_get(FTM1);
			encoder2 = -FTM_QUAD_get(FTM2);
			s = libutil::String::Format("Encoder1: %d",encoder1).c_str();
			Printline(m_lcd.FONT_H * 1, s);
			s = libutil::String::Format("Encoder2: %d",encoder2).c_str();
			Printline(m_lcd.FONT_H * 2, s);
			s = libutil::String::Format("Time: %d",libutil::Clock::Time()/1000).c_str();
			Printline(m_lcd.FONT_H * 3, s);

			///System loop - 1ms///
//			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
//				t = libutil::Clock::Time();
//
//				///Speed Control Output every 1ms///
//				SpeedControlOutput();
//
//				///Speed PID update every 100ms///
//				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
//				MoveMotor();
//
//			}

		}
		///////////////////////////Speed///////////////////////////
		break;
	case 5:
		///////////////////////////Camera///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Camera Mode");

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>=45){
				t = libutil::Clock::Time();
				PrintCam();
			}

			MoveMotor();

		}
		///////////////////////////Camera///////////////////////////
		break;
	case 6:
		///////////////////////////Parade///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Parade Mode");


		///////////////////////////Parade///////////////////////////
		break;
	case 7:
		Printline(m_lcd.FONT_H * 0, "Balance Only");

		///////////////////////////Balance Only///////////////////////////

		while(m_start_but.IsUp());

		m_control_speed1 = m_control_speed1 = m_turn_speed1 = m_turn_speed2 = 0;

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();




				if(t%150==0) {
					const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
					Printline(m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
					Printline(m_lcd.FONT_H * 2, s);
					s = libutil::String::Format("Angle: %d",(int)m_gyro).c_str();
					Printline(m_lcd.FONT_H * 3, s);
					s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
					Printline(m_lcd.FONT_H * 4, s);
				}


				///Update Gyro every 2ms///
				if(t % 2 == 0)	{
					mpu6050_update();
				}

				switch(t%2){
				case 0:
					BalanceControl();
					break;
				case 1:

					break;
				default:
					break;
				}

				MoveMotor();

				m_count++;
			}

		}
		///////////////////////////Balance Only///////////////////////////
		break;
	case 8:
		Printline(m_lcd.FONT_H * 0, "Camera Move");
		m_balance_speed1 = m_balance_speed2 = 2000;
		m_control_speed1 = m_control_speed2 = 0;

		///////////////////////////Camera Move///////////////////////////
		while (true)
		{
			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();
				if(t%150==0) {
					const char* s = libutil::String::Format("Turn: %d,%d",m_turn_speed1,m_turn_speed2).c_str();
					Printline(m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("Balance: %d, %d",m_balance_speed1, m_balance_speed2).c_str();
					Printline(m_lcd.FONT_H * 2, s);
					s = libutil::String::Format("Speed: %d, %d",m_control_speed1, m_control_speed2).c_str();
					Printline(m_lcd.FONT_H * 3, s);
					s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
					Printline(m_lcd.FONT_H * 4, s);
				}

				//if(t%45==0) {
					nt = libutil::Clock::Time();
					TurnControl();
					s = libutil::String::Format("T: %d",libutil::Clock::TimeDiff(libutil::Clock::Time(),nt)).c_str();
					Printline(m_lcd.FONT_H * 5, s);

				//}
				MoveMotor();
				m_count++;
			}
		}

		///////////////////////////Camera Move///////////////////////////
		break;
	case 9:
		Printline(m_lcd.FONT_H * 0, "Balance & Speed");

		///////////////////////////Balance & Speed///////////////////////////

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				SPEEDSETPOINT = 0;

	//				if(t%150==0 && autoprint) {
	//					const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
	//					Printline(m_lcd.FONT_H * 1, s);
	//					s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
	//					Printline(m_lcd.FONT_H * 2, s);
	//					s = libutil::String::Format("Angle: %d",(int)m_gyro).c_str();
	//					Printline(m_lcd.FONT_H * 3, s);
	//					s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
	//					Printline(m_lcd.FONT_H * 4, s);
	//				}

				///Speed Control Output every 1ms///
				SpeedControlOutput();


				///Update Gyro every 2ms///
				if(t%2==0){
					mpu6050_update();
				}

				if(t%5==0){
					BalanceControl();
				}


				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				MoveMotor();

				m_count++;
			}


		}
		///////////////////////////Balance & Speed///////////////////////////
		break;
	default:break;
	}
}

}
