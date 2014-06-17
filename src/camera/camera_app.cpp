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
#include <MK60_uart.h>
#include "mpu6050.h"
#include <libsc/com/lcd.h>
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>
#include <libsc/com/joystick.h>
#include <libsc/com/button.h>
#include "lcdmenu.h"

namespace camera
{

extern int mode_chosen = 0;

extern float b_kp[num_of_modes] = {3000.0, 3000.0, 3000.0};
extern float b_kd[num_of_modes] = {30000.0, 30000.0, 30000.0};
extern float b_ki[num_of_modes] = {0.0, 0.0, 0.0};

extern int16_t SPEEDSETPOINTS[num_of_modes] = {0, 95, 100};

extern float s_kp[num_of_modes] = {80.0, 80.0, 150.0};
extern float s_kd[num_of_modes] = {0.0, 0.0, 0.0};
extern float s_ki[num_of_modes] = {2.1, 2.1, 11.0};

extern float t_kp[num_of_modes] = {0.0, 0.02, 0.089};
extern float t_kd[num_of_modes] = {0.0, 0.0, 0.05};

CameraApp::CameraApp():
	m_gyro(0), m_balance_speed1(0), m_balance_speed2(0),
	m_control_speed1(0), m_control_speed2(0),
	m_balance_pid(SETPOINT, b_kp[mode_chosen], b_ki[mode_chosen], b_kd[mode_chosen]),
	m_speed_pid(SPEEDSETPOINTS[0], s_kp[mode_chosen], s_ki[mode_chosen], s_kd[mode_chosen]),
	m_count(0),
	m_total_speed1(0), m_total_speed2(0),
	m_encoder_2(0),
	prev_tempspeed(0), current_tempspeed(0),
	m_turn_speed1(0), m_turn_speed2(0),
	RightWhiteDot(0), LeftWhiteDot(0),
	m_helper(&m_car)
{
	gpio_init(PTB22, GPO, 0);
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf[0], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[1], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[2], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_acc_kf, 0.0005, 0.05, 0, 1);
	mpu6050_init();
	m_car.GetLcd()->Clear(WHITE);

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


	m_balance_speed1 = m_balance_speed2 = b_kp[mode_chosen] * e + b_kd[mode_chosen] * delta;

}


void CameraApp::SpeedControl(){
	static int32_t error, total_error = 0, delta_error, o_error = 0;
	int32_t encoder1 = FTM_QUAD_get(FTM1);
	int32_t encoder2 = -FTM_QUAD_get(FTM2);
	m_encoder_2 = (encoder1 + encoder2)/2;
	FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);

	error = SPEEDSETPOINTS[mode_chosen] - m_encoder_2;
	delta_error = error - o_error;
	total_error += error;
	o_error = error;

	prev_tempspeed = current_tempspeed;
	current_tempspeed = (int32_t) (s_kp[mode_chosen] * error + s_kd[mode_chosen] * delta_error + s_ki[mode_chosen] * total_error);

	if(m_encoder_2 > 350)
	{
		m_car.MoveMotor(0, 0);
		m_car.MoveMotor(1, 0);
		while(true);
	}
}

void CameraApp::SpeedControlOutput(){
    static int32_t speed_control_count;

    m_control_speed1 = m_control_speed2 = (current_tempspeed - prev_tempspeed) * (++speed_control_count) / SPEEDCONTROLPERIOD + prev_tempspeed;
    if(speed_control_count==SPEEDCONTROLPERIOD) speed_control_count = 0;
}

void CameraApp::ProcessImage(int n){

	const int start_row = CAM_H/3 * (n-1);
	const int end_row = CAM_H/3 * n;

	if(m_car.GetCamera()->IsImageReady())
	{
		gpio_set(PTB22, 1);
		src = m_car.GetCamera()->LockBuffer();
		for(int y=start_row; y<CAM_H/3; y++)
		{
			for(int x=0; x<end_row; x++)
			{
				if(m_helper.GetPixel(src, x, y) == WHITE_BYTE)
				{
					x>=(CAM_W/2) ? RightWhiteDot++ : LeftWhiteDot++;
				}
			}
		}
		gpio_set(PTB22, 0);
	}

}

void CameraApp::TurnControl(){

	static int areaPrevError = 0;

	int areaCurrentError = RightWhiteDot - LeftWhiteDot;			//http://notepad.cc/smartcar

	int degree = (int) round(
		(t_kp[mode_chosen] * areaCurrentError + t_kd[mode_chosen] * (areaPrevError - areaCurrentError)) /*+ (encoderCurrentError - encoderPrevError)*/
	);

	areaPrevError = areaCurrentError;

	degree = m_helper.Clamp(degree, -100, 100);

	m_turn_speed1 = - degree * 13;
	m_turn_speed2 = - m_turn_speed1;

	m_car.GetCamera()->UnlockBuffer();

}

void CameraApp::TurnControlOutput(){
	static int32_t turn_control_count;

	m_turn_speed1 = (current_turn - prev_turn) * (++turn_control_count) / TURNCONTROLPERIOD + prev_turn;
	m_turn_speed2 = -m_turn_speed1;
	if(turn_control_count==TURNCONTROLPERIOD) turn_control_count = 0;
}

void CameraApp::MoveMotor(){

	m_total_speed1 = m_balance_speed1 - m_control_speed1 + m_turn_speed1;
	m_total_speed2 = m_balance_speed2 - m_control_speed2 + m_turn_speed2;
	//sum+=(m_total_speed1 + m_total_speed2)/2;

	/*
	if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t) >= 50){
		t = libutil::Clock::Time();
		if(sum >= 50*3000){
			m_total_speed1 = m_total_speed2 = 0;
		}
		sum = 0;
	}
	*/

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

void CameraApp::AutoMode()
{
	uint16_t t = 0;
	uint16_t pt = 0;

	bool autoprint = false;

	libsc::Button* m_start_button = m_car.GetButton();

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "AUTO Mode");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, "Speed1: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 2, "Speed2: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 3, "Motor1: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 4, "Motor2: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 5, "Angle: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 6, "SSP: ");

	pt = libutil::Clock::Time();

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			if(t - pt > 5000) {
				mode_chosen = 1;
			}


			if(t%1500==0 && autoprint) {
				/*const char* s = libutil::String::Format("%06d",m_control_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%06d",m_control_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%06d",m_total_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%06d",m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 4, s);
				s = libutil::String::Format("%d",(int)m_gyro).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 5, s);
				s = libutil::String::Format("%d",SPEEDSETPOINT).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 6, s);*/
			}

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Turn Control Output every 1 ms///
			// TurnControlOutput();


			if(t%2==0){
				//mpu6050_update();
			}

			if(t%2==0){

				BalanceControl();
			}

			if(t - pt > 5000) {

				if(t%TURNCONTROLPERIOD==1){
					ProcessImage(1);
				}

				if(t%TURNCONTROLPERIOD==3){
					ProcessImage(2);
				}

				if(t%TURNCONTROLPERIOD==5){
					ProcessImage(3);
					TurnControl();
				}

			}

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==1){ SpeedControl(); }
			MoveMotor();

		}


		if(m_start_button->IsDown()) {
			autoprint = !autoprint;
			while(m_start_button->IsDown());
		}

	}

}

void CameraApp::PidMode()
{

}

void CameraApp::AccelAndGyroMode()
{
	uint16_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Accel & Gyro");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, "Angle: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 2, "Accel:");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 3, "A0:");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 4, "A1:");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 5, "A2:");
	gpio_init(PTD4, GPO, 1);

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
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%03d",(int)m_car.GetRawAngle()).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%03d", (int)angle[0]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%03d", (int)angle[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 4, s);
				s = libutil::String::Format("%03d", (int)angle[2]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 5, s);


			}


			m_count++;
		}

	}

}

void CameraApp::EncoderMode()
{
	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Encoder Mode");
	const char* s;
	int16 encoder1, encoder2;
	while (true)
	{
		encoder1 = FTM_QUAD_get(FTM1);
		encoder2 = -FTM_QUAD_get(FTM2);
		s = libutil::String::Format("Encoder1: %d",encoder1).c_str();
		m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
		s = libutil::String::Format("Encoder2: %d",encoder2).c_str();
		m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);
		s = libutil::String::Format("Time: %d",libutil::Clock::Time()/1000).c_str();
		m_helper.Printline(m_car.GetLcd()->FONT_H * 3, s);

		///System loop - 1ms///
		/*
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
			MoveMotor();

		}
		*/
	}

}

void CameraApp::CameraMode()
{
	uint16_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Camera Mode");

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>=1){
			t = libutil::Clock::Time();
			m_helper.PrintCam();
		}

	}
}

void CameraApp::ParadeMode()
{
	uint16_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Parade");

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			//SPEEDSETPOINT = 720;
			mode_chosen = 2;
			///Speed Control Output every 1ms///
			SpeedControlOutput();

			/*if(t%1000==0 && autoprint) {
				const char* s = libutil::String::Format("Speed: %04d,%04d",m_control_speed1,m_control_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("Motor: %04d, %04d",m_total_speed1, m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("Angle: %02d",(int)m_gyro).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("SSP: %d",SPEEDSETPOINTS[mode_chosen]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 4, s);
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

			/*
			if(m_start_button.IsDown()) {
				autoprint = !autoprint;
				while(m_start_button.IsDown());
			}
			*/
			m_count++;
		}


	}
}

void CameraApp::BalanceOnlyMode()
{
	uint16_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Balance Only");

	m_control_speed1 = m_control_speed1 = m_turn_speed1 = m_turn_speed2 = 0;

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			//if(t%150==0) {
			//	const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
			//	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
			//	s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
			//	m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);
			//	s = libutil::String::Format("Angle: %d",(int)m_gyro).c_str();
			//	m_helper.Printline(m_car.GetLcd()->FONT_H * 3, s);
			//	s = libutil::String::Format("SSP: %d",SPEEDSETPOINTS[mode_chosen]).c_str();
			//	m_helper.Printline(m_car.GetLcd()->FONT_H * 4, s);
			//}


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
}

void CameraApp::CameraMoveMode()
{
	uint16_t t = 0;

	bool autoprint = false;

	libsc::Button* m_start_button = m_car.GetButton();

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Camera Move");

	while (true)
	{
		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			/*
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
			*/
			//if(t - pt > 5000) {
				//SPEEDSETPOINT = 100;
				mode_chosen = 1;
			//}


			if(t%1500==0 && autoprint) {
				/*const char* s = libutil::String::Format("%06d",m_control_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%06d",m_control_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%06d",m_total_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%06d",m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 4, s);
				s = libutil::String::Format("%d",(int)m_gyro).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 5, s);
				s = libutil::String::Format("%d",SPEEDSETPOINT).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 6, s);*/
			}

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Turn Control Output every 1 ms///
			//TurnControlOutput();


			if(t%2==0){

				mpu6050_update();
			}

			if(t%2==0){

				BalanceControl();
			}

			//if(t - pt > 5000) {

				if(t%TURNCONTROLPERIOD==1){
					ProcessImage(1);
				}

				if(t%TURNCONTROLPERIOD==3){
					ProcessImage(2);
				}

				if(t%TURNCONTROLPERIOD==5){
					ProcessImage(3);
					TurnControl();
				}

			//}

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==1){ SpeedControl(); }
			m_balance_speed1 = m_balance_speed2 = 0;
			m_control_speed1 = m_control_speed2 = 0;
			MoveMotor();

		}

		if(m_start_button->IsDown()) {
			autoprint = !autoprint;
			while(m_start_button->IsDown());
		}
	}

}

void CameraApp::BalanceAndSpeedMode()
{
	uint16_t t = 0;

	bool autoprint = false;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Balance & Speed");

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

//				SPEEDSETPOINT = 0;
			///Speed Control Output every 1ms///
			SpeedControlOutput();

			if(t%2000==0 && autoprint) {
				const char* s = libutil::String::Format("%04d, %04d",m_total_speed1, m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);

			}



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

		}
	}

}

void CameraApp::MoveMotorMode()
{

}

void CameraApp::UartMode()
{

}

void CameraApp::CalGyroMode()
{
	libsc::Joystick* m_joystick = m_car.GetJoystick();

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Balance & Speed");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, "Calibrating gyro...");
	gyro_cal();
	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, "Gyro calibrated.");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 2, "Press joystick");
	//if(m_joystick->GetState()==libsc::Joystick::SELECT) mode=1;
}

void CameraApp::SpeedModeOne()
{
	uint16_t t = 0;

	bool autoprint = false;

	libsc::Button* m_start_button = m_car.GetButton();

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Speed");

	while (true)
	{
		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();
			m_balance_speed1 = m_balance_speed2 = m_turn_speed1 = m_turn_speed2 = 0;
			//SPEEDSETPOINT = 0;

			if(t%1000==0 && autoprint) {
				const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed1,m_control_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("Motor: %d, %d",m_total_speed1, m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);

				s = libutil::String::Format("SSP: %d",SPEEDSETPOINTS[mode_chosen]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 4, s);
			}

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
			MoveMotor();
			if(m_start_button->IsDown()) {
				autoprint = !autoprint;
				while(m_start_button->IsDown());
			}
			m_count++;
		}

	}

}

void CameraApp::SpeedModeTwo()
{
	uint16_t t = 0;
	uint16_t pt = 0;

	libsc::Button* m_start_button = m_car.GetButton();

	bool autoprint = false;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Speed 2");

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			/*
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
			*/
			if(t - pt > 5000) {
				//SPEEDSETPOINT = 100;
				mode_chosen = 1;
			}


			if(t%1500==0 && autoprint) {
				/*const char* s = libutil::String::Format("%06d",m_control_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%06d",m_control_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%06d",m_total_speed1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%06d",m_total_speed2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 4, s);
				s = libutil::String::Format("%d",(int)m_gyro).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 5, s);
				s = libutil::String::Format("%d",SPEEDSETPOINT).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 6, s);*/
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



			//if(t%TURNCONTROLPERIOD==0){ TurnControl(); }

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==0){ SpeedControl(); }
			MoveMotor();

		}


		if(m_start_button->IsDown()) {
			autoprint = !autoprint;
			while(m_start_button->IsDown());
		}

	}

}

void CameraApp::TimeMeasurementMode()
{
	uint16_t t = 0;
	uint16_t pt = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Time measurement");

	pt = libutil::Clock::Time();
	gpio_init(PTD4, GPO, 1);
	while (true)
	{
		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			//gpio_set(PTD4,1);
			//SpeedControlOutput();
			//gpio_set(PTD4,0);
			//DELAY_MS(5);
			//gpio_set(PTD4,1);
			//mpu6050_update();
			//gpio_set(PTD4,0);
			//DELAY_MS(5);
			//gpio_set(PTD4,1);
			//BalanceControl();
			//gpio_set(PTD4,0);
			//DELAY_MS(5);
			//gpio_set(PTD4,1);
			//TurnControl();
			//gpio_set(PTD4,0);
			//DELAY_MS(5);
			//gpio_set(PTD4,1);
			//SpeedControl();
			//gpio_set(PTD4,0);
			//DELAY_MS(5);
			gpio_set(PTD4,1);
			MoveMotor();
			gpio_set(PTD4,0);
			DELAY_MS(5);

		}

	}

}

void CameraApp::SpeedToMotorMode()
{
	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Speed to Motor");
}

void CameraApp::Run()
{
	m_car.GetLcd()->Clear(WHITE);

	char* title = "Mode:";
	char* choices[16] = {
		"Auto",				//1
		"PID",				//2
		"Accel & Gyro",		//3
		"Encoder",			//4
		"Camera",			//5
		"Parade",			//6
		"Balance Only",		//7
		"Camera Move",		//8
		"Balance & Speed",	//9
		"Move motor",		//10
		"UART",				//11
		"Cal Gyro",			//12
		"Speed mode",		//13
		"Speed 2 mode",		//14
		"Time measurement",	//15
		"Speed to Motor"	//16
	};

	LcdMenu lcdmenu(&m_car, title, choices, 16);
	lcdmenu.CreateMenu();
	lcdmenu.WaitForSelection();

	m_car.GetLcd()->Clear(WHITE);

	switch(lcdmenu.GetSelectedChoice()){
		case 1:
			AutoMode();
			break;
		case 2:
			PidMode();
			break;
		case 3:
			AccelAndGyroMode();
			break;
		case 4:
			EncoderMode();
			break;
		case 5:
			CameraMode();
			break;
		case 6:
			ParadeMode();
			break;
		case 7:
			BalanceOnlyMode();
			break;
		case 8:
			CameraMoveMode();
			break;
		case 9:
			BalanceAndSpeedMode();
			break;
		case 10:
			MoveMotorMode();
			break;
		case 11:
			UartMode();
			break;
		case 12:
			CalGyroMode();
			break;
		case 13:
			SpeedModeOne();
			break;
		case 14:
			SpeedModeTwo();
			break;
		case 15:
			TimeMeasurementMode();
			break;
		case 16:
			SpeedToMotorMode();
			break;
		default:
			break;
	}
}

}
