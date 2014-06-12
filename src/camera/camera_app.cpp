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
	mpu6050_init();
	m_lcd.Clear(WHITE);

}

CameraApp::~CameraApp()
{

}
int abs(const int num)
{
	return num>0 ? num : -1*num;
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

}


void CameraApp::SpeedControl(){
	static int32_t error, total_error, delta_error, o_error = 0;
	int32_t encoder1 = FTM_QUAD_get(FTM1);
	int32_t encoder2 = -FTM_QUAD_get(FTM2);
	m_encoder_2 = (encoder1 + encoder2)/2;
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
			if(m_car.GetPixel(src, x, y) == WHITE_BYTE)
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
		const Byte *buf = m_car.ExpandPixel(src, i);
		m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
	}

	//---------------------Edge Detection---------------------//
	int LeftEdgeX, CenterX[CAM_H], RightEdgeX;
	memset(CenterX, -1, CAM_H);

	for(int y=CAM_H-1; y>=0; y--)
	{
		LeftEdgeX = -1;
		RightEdgeX = -1;
		/*
		int BlackCount = 0;
		int BlackSum = 0;

		for(int x=0; x<CAM_W; x++)
		{
			if(m_car.GetPixel(src, x, y) == BLACK_BYTE)
			{
				BlackCount++;
				BlackSum+=x;
			}
		}

		CenterX[y] = (int) round(BlackSum / BlackCount);

		if(CenterX[y] == BLACK_BYTE && y+1<CAM_H && y+2<CAM_H)
			CenterX[y] = CenterX[y+1] + (CenterX[y+1] - CenterX[y+2]);
*/

		if(y+2>=CAM_H)
			CenterX[y] = CenterX[y+1] + (CenterX[y+1] - CenterX[y+2]);
		else
			CenterX[y] = CAM_W/2;

		CenterX[y] = m_car.Clamp(CenterX[y]);

		int x = CenterX[y];

		if(
			m_car.GetPixel(src, x, y)==BLACK_BYTE
			&& m_car.GetPixel(src, m_car.Clamp(x-1), y)==BLACK_BYTE
			&& m_car.GetPixel(src, m_car.Clamp(x+1), y)==BLACK_BYTE
		)
		{
			const char* s = libutil::String::Format("%d, %d, %d", x, x-1, x+1).c_str();
			Printline(m_lcd.FONT_H * 7+30, s);
			break;
		}

		for(int x=CenterX[y]; x>=0; x--)
		{
			if(
				x-1>=0 && x-2>=0 && x-3>=0
				&& m_car.GetPixel(src, x, y) == WHITE_BYTE
				&& m_car.GetPixel(src, x-1, y) == BLACK_BYTE
				&& m_car.GetPixel(src, x-2, y) == BLACK_BYTE
				&& m_car.GetPixel(src, x-3, y) == BLACK_BYTE
			)
			{
				LeftEdgeX = x-1;
				break;
			}
		}

		for(int x=CenterX[y]; x<CAM_W; x++)
		{
			if(
				x+1<CAM_W && x+2<CAM_W && x+3<CAM_W
				&& m_car.GetPixel(src, x, y) == WHITE_BYTE
				&& m_car.GetPixel(src, x+1, y) == BLACK_BYTE
				&& m_car.GetPixel(src, x+2, y) == BLACK_BYTE
				&& m_car.GetPixel(src, x+3, y) == BLACK_BYTE
			)
			{
				RightEdgeX = x+1;
				break;
			}
		}

		if(LeftEdgeX==-1 || RightEdgeX==-1)
		{
			if(LeftEdgeX==-1)
				LeftEdgeX = 0;
			if(RightEdgeX==-1)
				RightEdgeX = CAM_W-1;

		}

		CenterX[y] = (int) round((LeftEdgeX + RightEdgeX)/2);

		m_lcd.DrawPixel((CAM_H - 1) - y, LeftEdgeX, libutil::GetRgb565(0xFF, 0x00, 0x00));
		m_lcd.DrawPixel((CAM_H - 1) - y, RightEdgeX, libutil::GetRgb565(0x00, 0xFF, 0x00));
		m_lcd.DrawPixel((CAM_H - 1) - y, CenterX[y], libutil::GetRgb565(0x00, 0x00, 0xFF));
	}
	//---------------------Edge Detection---------------------//


	m_car.GetCamera()->UnlockBuffer();
	uint16_t sec = libutil::Clock::Time()/1000;
	const char* s = libutil::String::Format("Time: %02d",sec).c_str();
	Printline(m_lcd.FONT_H * 7, s);
	//DELAY_MS(10);
}

void CameraApp::TurnControlOutput(){

}

void CameraApp::MoveMotor(){
	static uint16_t t = 0;
	static uint32_t sum = 0;
	m_total_speed1 = m_balance_speed1 - m_control_speed1 + m_turn_speed1;
	m_total_speed2 = m_balance_speed2 - m_control_speed2 + m_turn_speed2;
	//sum+=(m_total_speed1 + m_total_speed2)/2;
	m_total_speed1 = m_total_speed1 * 90 / 100;

	/*if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t) >= 50){
		t = libutil::Clock::Time();
		if(sum >= 50*3000){
			m_total_speed1 = m_total_speed2 = 0;
		}
		sum = 0;
	}*/

	if(m_total_speed1 > 0){
		m_dir1 = true;
	}else{
		m_dir1 = false;
	}

	if(m_total_speed2 > 0){
		m_dir2 = true;
	}else{
		m_dir2 = false;
	}

	m_car.MotorDir(0, m_dir1); 			////Right Motor - True Backward  -  False Forward
	m_car.MoveMotor(0,(uint16_t) abs(m_total_speed1));

	m_car.MotorDir(1, !m_dir2);			////Left Motor - False Backward  -  True Forward
	m_car.MoveMotor(1,(uint16_t) abs(m_total_speed2));
}

void CameraApp::Printline(uint8_t* x, uint8_t y, const char* s, const uint16_t TXT_COLOR, const uint16_t BG_COLOR){

	while(*s){
		if(y==0) m_lcd.DrawChar(*x, y, *s, WHITE, BLACK);
		else m_lcd.DrawChar(*x, y, *s, TXT_COLOR, BG_COLOR);
		*x+=m_lcd.FONT_W;
		s++;
	}

}

void CameraApp::Printline(uint8_t y, const char* s, bool inverted){
	uint8_t x=0;
	Printline(&x, y, s, WHITE, INVERTED_BG_COLOR);
	for(int i=x; i<m_lcd.W; i++) m_lcd.DrawChar(i, y, ' ', WHITE, INVERTED_BG_COLOR);

}

void CameraApp::Printline(uint8_t y, const char* s){
	uint8_t x=0;
	if(y==0){
		for(int i=0; i<x; i++) m_lcd.DrawChar(i, y, ' ', WHITE, BLACK);
	}else{
		for(int i=0; i<x; i++) m_lcd.DrawChar(i, y, ' ', BLACK, WHITE);
	}
	Printline(&x, y, s);
	if(y==0){
		for(int i=x; i<m_lcd.W; i++) m_lcd.DrawChar(i, y, ' ', WHITE, BLACK);
	}else{
		for(int i=x; i<m_lcd.W; i++) m_lcd.DrawChar(i, y, ' ', BLACK, WHITE);
	}
}

void CameraApp::Run()
{
	libsc::Button m_start_but(0);
	libsc::Joystick m_joystick(0);


	int maxh = m_lcd.H/m_lcd.FONT_H - 1;
	int maxw = m_lcd.W/m_lcd.FONT_W - 1;

	m_lcd.Clear(WHITE);


	int mode = -1;
	int ptr_pos = 1;
	int ptr_output_pos = 1;
	int maxchoices = 16;
	int viewport[2] = {1, maxh};


	char** s = new char*[maxchoices+1];
	s[0] = "Mode:";
	s[1] = "Auto";
	s[2] = "PID";
	s[3] = "Accel & Gyro";
	s[4] = "Encoder";
	s[5] = "Camera";
	s[6] = "Parade";
	s[7] = "Balance Only";
	s[8] = "Camera Move";
	s[9] = "Balance & Speed";
	s[10] = "Move motor";
	s[11] = "UART";
	s[12] = "Cal Gyro";
	s[13] = "Speed mode";
	s[14] = "Speed 2 mode";
	s[15] = "Time measurement";
	s[16] = "Speed to Motor";

	for(int i=0; i<=maxh; i++) Printline(m_lcd.FONT_H * i, s[i]);
	uint8_t mx;

	Printline(m_lcd.FONT_H * ptr_output_pos, s[viewport[0]+ptr_output_pos-1], true);

	while(mode==-1){
		mx = m_lcd.FONT_W * 6;
		Printline( mx , 0, libutil::String::Format("%02d/%02d",ptr_pos,maxchoices).c_str());
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
					for(int i=1; i<=maxh; i++) {
						//Printline(m_lcd.FONT_H * i, "                ");
						if(ptr_output_pos == i) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1], true);
						else Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
					}
				}

			}else{
				ptr_pos = 1;
				ptr_output_pos = 1;
				viewport[0] = 1;
				viewport[1] = maxh;
				for(int i=1; i<=maxh; i++) {
					//Printline(m_lcd.FONT_H * i, "                ");
					if(ptr_output_pos == i) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1], true);
					else Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
				}
			}
			if(ptr_output_pos != 1) Printline(m_lcd.FONT_H * (ptr_output_pos-1), s[viewport[0]+ptr_output_pos-1-1]);
			Printline(m_lcd.FONT_H * ptr_output_pos, s[viewport[0]+ptr_output_pos-1], true);
			//PrintPtr(ptr_output_pos * m_lcd.FONT_H);
			DELAY_MS(100);
			break;

		case libsc::Joystick::UP:
			if(ptr_pos - 1 >= 1){
				--ptr_pos;
				--ptr_output_pos;
				ptr_output_pos = ptr_output_pos < 1 ? 1 : ptr_output_pos;

				if(ptr_pos < viewport[0]){
					viewport[0]--;
					viewport[1]--;
					for(int i=1; i<=maxh; i++) {
						//Printline(m_lcd.FONT_H * i, "                ");
						if(ptr_output_pos == i) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1], true);
						else Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
					}
				}

			}else{
				ptr_pos = maxchoices;
				ptr_output_pos = maxh;
				viewport[0] = maxchoices - maxh + 1;
				viewport[1] = maxchoices;
				for(int i=1; i<=maxh; i++) {
					//Printline(m_lcd.FONT_H * i, "                ");
					if(ptr_output_pos == i) Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1], true);
					else Printline(m_lcd.FONT_H * i, s[viewport[0]+i-1]);
				}
			}
			if(ptr_output_pos != maxh) Printline(m_lcd.FONT_H * (ptr_output_pos+1), s[viewport[0]+ptr_output_pos-1+1]);
			Printline(m_lcd.FONT_H * ptr_output_pos, s[viewport[0]+ptr_output_pos-1], true);
			//PrintPtr(ptr_output_pos * m_lcd.FONT_H);
			DELAY_MS(100);

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
	m_lcd.Clear(WHITE);
	switch(mode){
	case 1:
		Printline(m_lcd.FONT_H * 0, "AUTO Mode");
		Printline(m_lcd.FONT_H * 1, "Speed1: ");
		Printline(m_lcd.FONT_H * 2, "Speed2: ");
		Printline(m_lcd.FONT_H * 3, "Motor1: ");
		Printline(m_lcd.FONT_H * 4, "Motor2: ");
		Printline(m_lcd.FONT_H * 5, "Angle: ");
		Printline(m_lcd.FONT_H * 6, "SSP: ");
		///////////////////////////AUTO///////////////////////////

		//while(m_start_but.IsUp());

		pt = libutil::Clock::Time();

		while (true)
		{
			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				/*if(t - pt> 5000){
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
				}*/

				SPEEDSETPOINT = 300;


				if(t%1500==0 && autoprint) {
					const char* s = libutil::String::Format("%06d",m_control_speed1).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("%06d",m_control_speed2).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 2, s);
					s = libutil::String::Format("%06d",m_total_speed1).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 3, s);
					s = libutil::String::Format("%06d",m_total_speed2).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 4, s);
					s = libutil::String::Format("%d",(int)m_gyro).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 5, s);
					s = libutil::String::Format("%d",SPEEDSETPOINT).c_str();
					Printline(m_lcd.FONT_W * 8, m_lcd.FONT_H * 6, s);
				}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				//TurnControlOutput();


				if(t%2==0){

					mpu6050_update();
				}

				if(t%2==1){

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
		Printline(m_lcd.FONT_H * 1, "Angle: ");
		Printline(m_lcd.FONT_H * 2, "Accel:");
		Printline(m_lcd.FONT_H * 3, "A0:");
		Printline(m_lcd.FONT_H * 4, "A1:");
		Printline(m_lcd.FONT_H * 5, "A2:");
		gpio_init(PTD4, GPO, 1);
		///////////////////////////Accel & Gyro///////////////////////////

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				///Update Gyro every 2ms///
				if(t % 2 == 0)	{
					gpio_set(PTD4, 1);
					mpu6050_update();
					gpio_set(PTD4, 0);
					BalanceControl();
				}


				if(t%2000==0) {

					const char* s = libutil::String::Format("%03d",(int)m_gyro).c_str();
					Printline(m_lcd.FONT_W * 7, m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("%03d",(int)m_car.GetRawAngle()).c_str();
					Printline(m_lcd.FONT_W * 7, m_lcd.FONT_H * 2, s);
					s = libutil::String::Format("%03d", (int)angle[0]).c_str();
					Printline(m_lcd.FONT_W * 7, m_lcd.FONT_H * 3, s);
					s = libutil::String::Format("%03d", (int)angle[1]).c_str();
					Printline(m_lcd.FONT_W * 7, m_lcd.FONT_H * 4, s);
					s = libutil::String::Format("%03d", (int)angle[2]).c_str();
					Printline(m_lcd.FONT_W * 7, m_lcd.FONT_H * 5, s);


				}


				m_count++;
			}

		}
		///////////////////////////Gyro///////////////////////////
		break;
	case 4:
		///////////////////////////Encoder Mode///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Encoder Mode");
		const char* s;
		int16 encoder1, encoder2;
		while (true)
		{
			encoder1 = FTM_QUAD_get(FTM1);
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
		///////////////////////////Encoder Mode///////////////////////////
		break;
	case 5:
		///////////////////////////Camera///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Camera Mode");

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>=100){
				t = libutil::Clock::Time();
				PrintCam();
			}

			MoveMotor();

		}
		///////////////////////////Camera///////////////////////////
		break;
	case 6:
		Printline(m_lcd.FONT_H * 0, "Parade");

				///////////////////////////Balance & Speed 2///////////////////////////

				while (true)
				{

					///System loop - 1ms///
					if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
						t = libutil::Clock::Time();

						SPEEDSETPOINT = 720;
						///Speed Control Output every 1ms///
						SpeedControlOutput();

						/*if(t%1000==0 && autoprint) {
							const char* s = libutil::String::Format("Speed: %04d,%04d",m_control_speed1,m_control_speed2).c_str();
							Printline(m_lcd.FONT_H * 1, s);
							s = libutil::String::Format("Motor: %04d, %04d",m_total_speed1, m_total_speed2).c_str();
							Printline(m_lcd.FONT_H * 2, s);
							s = libutil::String::Format("Angle: %02d",(int)m_gyro).c_str();
							Printline(m_lcd.FONT_H * 3, s);
							s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
							Printline(m_lcd.FONT_H * 4, s);
						}*/




						///Update Gyro every 2ms///
						/*if(t%2==0){
							mpu6050_update();
						}

						if(t%2==0){
							BalanceControl();
						}*/
						switch(t%2){
						case 0:
							mpu6050_update();
							break;
						case 1:
							BalanceControl();
							break;
						}

						///Speed PID update every 20ms///
						if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
						MoveMotor();

						/*if(m_start_but.IsDown()) {
							autoprint = !autoprint;
							while(m_start_but.IsDown());
						}*/
						m_count++;
					}


				}
		///////////////////////////Balance & Speed 2///////////////////////////
		break;
	case 7:
		Printline(m_lcd.FONT_H * 0, "Balance Only");

		///////////////////////////Balance Only///////////////////////////

		m_control_speed1 = m_control_speed1 = m_turn_speed1 = m_turn_speed2 = 0;

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();




//				if(t%150==0) {
//					const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
//					Printline(m_lcd.FONT_H * 1, s);
//					s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
//					Printline(m_lcd.FONT_H * 2, s);
//					s = libutil::String::Format("Angle: %d",(int)m_gyro).c_str();
//					Printline(m_lcd.FONT_H * 3, s);
//					s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
//					Printline(m_lcd.FONT_H * 4, s);
//				}


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
				///Speed Control Output every 1ms///
				SpeedControlOutput();

				/*if(t%1000==0 && autoprint) {
					const char* s = libutil::String::Format("Speed: %04d,%04d",m_control_speed1,m_control_speed2).c_str();
					Printline(m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("Motor: %04d, %04d",m_total_speed1, m_total_speed2).c_str();
					Printline(m_lcd.FONT_H * 2, s);
					s = libutil::String::Format("Angle: %02d",(int)m_gyro).c_str();
					Printline(m_lcd.FONT_H * 3, s);
					s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
					Printline(m_lcd.FONT_H * 4, s);
				}*/




				///Update Gyro every 2ms///
				/*if(t%2==0){
					mpu6050_update();
				}

				if(t%2==0){
					BalanceControl();
				}*/
				switch(t%2){
				case 0:
					mpu6050_update();
					break;
				case 1:
					BalanceControl();
					break;
				}

				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				MoveMotor();

				/*if(m_start_but.IsDown()) {
					autoprint = !autoprint;
					while(m_start_but.IsDown());
				}*/
				m_count++;
			}


		}
		///////////////////////////Balance & Speed///////////////////////////
		break;
	case 12:
		Printline(m_lcd.FONT_H * 0, "Balance & Speed");
		Printline(m_lcd.FONT_H * 1, "Calibrating gyro...");
		gyro_cal();
		Printline(m_lcd.FONT_H * 1, "Gyro calibrated.");
		Printline(m_lcd.FONT_H * 2, "Press joystick");
		if(m_joystick.GetState()==libsc::Joystick::SELECT) mode=1;
		break;
	case 13:
		///////////////////////////Speed///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Speed");
		while (true)
		{
			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();
				m_balance_speed1 = m_balance_speed2 = m_turn_speed1 = m_turn_speed2 = 0;
				SPEEDSETPOINT = 0;

				if(t%1000==0 && autoprint) {
					const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
					Printline(m_lcd.FONT_H * 1, s);
					s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
					Printline(m_lcd.FONT_H * 2, s);

					s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
					Printline(m_lcd.FONT_H * 4, s);
				}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				MoveMotor();
				if(m_start_but.IsDown()) {
					autoprint = !autoprint;
					while(m_start_but.IsDown());
				}
				m_count++;
			}


		}
		///////////////////////////Speed///////////////////////////
		break;
	case 14:
		Printline(m_lcd.FONT_H * 0, "Speed 2");
			///////////////////////////Speed2///////////////////////////
			while (true)
			{

				///System loop - 1ms///
				if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
					t = libutil::Clock::Time();

					SPEEDSETPOINT = 200;

					if(t%1000==0 && autoprint) {
						const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
						Printline(m_lcd.FONT_H * 1, s);
						s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
						Printline(m_lcd.FONT_H * 2, s);

						s = libutil::String::Format("SSP: %d",SPEEDSETPOINT).c_str();
						Printline(m_lcd.FONT_H * 4, s);
					}

					///Speed Control Output every 1ms///
					SpeedControlOutput();

					///Speed PID update every 100ms///
					if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
					MoveMotor();
					if(m_start_but.IsDown()) {
						autoprint = !autoprint;
						while(m_start_but.IsDown());
					}
					m_count++;
				}


			}
			///////////////////////////Speed2///////////////////////////
			break;
	case 15:
			Printline(m_lcd.FONT_H * 0, "Time measurement");

			///////////////////////////Time measurement///////////////////////////

			pt = libutil::Clock::Time();
			gpio_init(PTD4, GPO, 1);
			while (true)
			{
				///System loop - 1ms///
				if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
					t = libutil::Clock::Time();

//					gpio_set(PTD4,1);
//					SpeedControlOutput();
//					gpio_set(PTD4,0);
//					DELAY_MS(5);
//					gpio_set(PTD4,1);
//					mpu6050_update();
//					gpio_set(PTD4,0);
//					DELAY_MS(5);
//					gpio_set(PTD4,1);
//					BalanceControl();
//					gpio_set(PTD4,0);
//					DELAY_MS(5);
//					gpio_set(PTD4,1);
//					TurnControl();
//					gpio_set(PTD4,0);
//					DELAY_MS(5);
//					gpio_set(PTD4,1);
//					SpeedControl();
//					gpio_set(PTD4,0);
//					DELAY_MS(5);
					gpio_set(PTD4,1);
					MoveMotor();
					gpio_set(PTD4,0);
					DELAY_MS(5);

				}


			}
			///////////////////////////Time measurement///////////////////////////

			break;
	case 16:
		Printline(m_lcd.FONT_H * 0, "Speed to Motor");

		break;
	default:break;
	}
}

}
