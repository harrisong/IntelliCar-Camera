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
#include <libutil/string.h>
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>
#include "camera/camera_app.h"
#include <MK60_gpio.h>
#include <MK60_adc.h>
#include <MK60_i2c.h>
#include <MK60_uart.h>
#include "mpu6050.h"
#include <libsc/com/lcd.h>
#include <libsc/com/joystick.h>
#include <libsc/com/button.h>
#include "lcdmenu.h"

using namespace libutil;

namespace camera
{

float b_kp[3] = {900.0, 900.0, 900.0};
float b_ki[3] = {0.0, 0.0, 0.0};
float b_kd[3] = {27000.0, 27000.0, 27000.0};

int16_t SPEED_SETPOINTS[3] = {0, 85, 100};

float s_kp[3] = {110.0, 110.0, 150.0};
float s_ki[3] = {9.0, 9.0, 11.0};
float s_kd[3] = {0.0, 0.0, 0.0};

float t_kp[3] = {1.0, 1.0, 1.0};
float t_ki[3] = {0.0, 0.0, 0.0};
float t_kd[3] = {0.2, 0.2, 0.2};

CameraApp::CameraApp():
	m_helper(&m_car),
	m_balance_speed{0, 0},
	m_control_speed{0, 0},
	m_turn_speed{0, 0},
	m_total_speed{0, 0},
	m_dir{false, false},
	white_dot{0, 0},
	m_speed_pid(SPEED_SETPOINTS[0], s_kp, s_ki, s_kd, 3, 1), //SETPOINT, kp array, ki array, kd array, max_mode, initital mode (from 1 to max_mode);
	m_turn_pid(TURN_SETPOINT, t_kp, t_ki, t_kd, 3, 1),
	m_balance_pid(BALANCE_SETPOINT, b_kp, b_ki, b_kd, 3, 1),
	speed_smoothing(SPEEDCONTROLPERIOD),
	speed_input_smoothing(3000),
	turn_smoothing(TURNCONTROLPERIOD),
	m_gyro(0),
	m_encoder_2(0),
	m_count(0),
	num_finished_row(0),
	src(NULL),
	e_stop(0)
{
	gpio_init(PTA11, GPO, 1);
	DELAY_MS(500);
	gpio_set(PTA11, 0);

	gpio_init(PTB22, GPO, 0);
	gpio_init(PTD2, GPO, 0);
	gpio_set(PTD2, 0);
	gpio_init(PTD3, GPO, 0);
	gpio_set(PTD3, 0);
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf[0], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[1], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[2], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_acc_kf, 0.0005, 0.05, 0, 1);
	mpu6050_init();
	m_car.GetLcd()->Clear(WHITE);

	tunableints[0] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("bkp", TunableInt::REAL,
			TunableInt::AsUnsigned(b_kp[1]));
	tunableints[1] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("bkd", TunableInt::REAL,
				TunableInt::AsUnsigned(b_kd[1]));
	tunableints[2] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("bki", TunableInt::REAL,
					TunableInt::AsUnsigned(b_ki[1]));
	tunableints[3] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("skp", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kp[1]));
	tunableints[4] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("skd", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kd[1]));
	tunableints[5] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("ski", TunableInt::REAL,
						TunableInt::AsUnsigned(s_ki[1]));
	tunableints[6] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("tkp", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kp[1]));
	tunableints[7] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("tkd", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kd[1]));
	tunableints[8] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("speed", TunableInt::INTEGER,
						TunableInt::AsSigned(SPEED_SETPOINTS[1]));
	tunableints[9] = TunableIntManager<10>::GetInstance(m_car.GetUart())->Register("estop", TunableInt::INTEGER,
						TunableInt::AsSigned(-1));

	TunableIntManager<10>::GetInstance(m_car.GetUart())->Start();

}

CameraApp::~CameraApp()
{

}

void CameraApp::eStop(){
	m_car.MoveMotor(0, 0);
	m_car.MoveMotor(1, 0);

	angle[0] = 18;
	m_speed_pid.ResetError();
	m_speed_pid.SetSetPoint(0);
	m_balance_pid.ResetError();
	m_turn_pid.ResetError();

	while(m_car.GetJoystick()->GetState() != libsc::Joystick::SELECT);

	e_stop++;
}

void CameraApp::BalanceControl()
{
	///Get values from gyro///
	m_car.AccelRefresh();

	float acc = m_car.GetRawAngle();
	kalman_filtering(&m_acc_kf, &acc, 1);
	m_gyro = 1/TIMECONST * acc + (1- 1/TIMECONST) * angle[0];
//	m_gyro = angle[0];

	m_balance_pid.UpdateCurrentError(m_gyro);

	m_balance_speed[0] = m_balance_speed[1] = m_balance_pid.Proportional() + m_balance_pid.Derivative();

	m_balance_pid.UpdatePreviousError();
}


void CameraApp::SpeedControl(){
#ifndef LIBSC_USE_K60_ENCODERS
	return;
#endif

	int32_t encoder1 = FTM_QUAD_get(FTM1);
	int32_t encoder2 = -FTM_QUAD_get(FTM2);

	m_encoder_2 = (encoder1 + encoder2)/2;

	FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);

	m_speed_pid.UpdateCurrentError( m_encoder_2 );

	speed_smoothing.UpdateCurrentOutput( (int32_t) (m_speed_pid.Proportional() + m_speed_pid.Integral() + m_speed_pid.Derivative()) );

	m_speed_pid.UpdatePreviousError();

//	if(m_encoder_2 > 500)
//	{
//		eStop();
//	}
}


void CameraApp::SpeedControlOutput(){

    m_control_speed[0] = m_control_speed[1] = speed_smoothing.SmoothingOutput();
}

void CameraApp::ProcessImage(){

	const int start_row = num_finished_row;
	const int end_row = num_finished_row+15;

	if(m_car.GetCamera()->IsImageReady())
	{
//		gpio_set(PTB22, 1);
		src = m_car.GetCamera()->LockBuffer();
		for(int y=start_row; y<end_row; y++)
		{
			for(int x=0; x<CAM_W; x++)
			{
				if(m_helper.GetPixel(src, x, y) == WHITE_BYTE)
				{
					x>=(CAM_W/2) ? white_dot[1]++ : white_dot[0]++;
				}
			}
		}

//		gpio_set(PTB22, 0);
	}

}

void CameraApp::TurnControl(){
	gpio_set(PTB22, 1);
	m_turn_pid.UpdateCurrentError(white_dot[1] - white_dot[0]);

	white_dot[0] = white_dot[1] = 0;

	int degree = (int) round(
		- m_turn_pid.Proportional() + -1 * m_turn_pid.Derivative()
	);

	m_turn_pid.UpdatePreviousError();

//	degree = m_helper.Clamp(degree, -100, 100);

	//turn_smoothing.UpdateCurrentOutput(- degree * 13);
//	printf("A: %f\n", angle[0]);
	m_turn_speed[0] = - degree * 10;
	m_turn_speed[1] = - m_turn_speed[0];

	m_car.GetCamera()->UnlockBuffer();
	gpio_set(PTB22, 0);
}

void CameraApp::TurnControlOutput(){
	m_turn_speed[0] = turn_smoothing.SmoothingOutput();
	m_turn_speed[1] = -m_turn_speed[0];
}

void CameraApp::MoveMotor(){

	m_total_speed[0] = m_balance_speed[0] - m_control_speed[0] + m_turn_speed[0];
	m_total_speed[1] = m_balance_speed[1] - m_control_speed[1] + m_turn_speed[1];
	//sum+=(m_total_speed[0] + m_total_speed[1])/2;

	/*
	if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t) >= 50){
		t = libutil::Clock::Time();
		if(sum >= 50*3000){
			m_total_speed[0] = m_total_speed[1] = 0;
		}
		sum = 0;
	}
	*/

	m_dir[0] = m_total_speed[0] > 0 ? true : false;
	m_dir[1] = m_total_speed[1] > 0 ? true : false;

	m_car.MotorDir(0, !m_dir[0]); 			////Right Motor - True Backward  -  False Forward
	m_car.MoveMotor(0,(uint16_t) m_helper.abs(m_total_speed[0]));

	m_car.MotorDir(1, m_dir[1]);			////Left Motor - False Backward  -  True Forward
	m_car.MoveMotor(1,(uint16_t) m_helper.abs(m_total_speed[1]));
}

void CameraApp::AutoMode()
{
	uint32_t t = 0, set_time = 0;
	uint32_t dt = 0;
	uint32_t pt = libutil::Clock::Time();

	bool autoprint = false;

	libsc::Button* m_start_button = m_car.GetButton();

	/*m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "AUTO Mode");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 1, "Speed1: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 2, "Speed2: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 3, "Motor1: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 4, "Motor2: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 5, "Angle: ");
	m_helper.Printline(m_car.GetLcd()->FONT_H * 6, "SSP: ");*/
	m_balance_pid.SetMode(2);
	m_turn_pid.SetMode(2);
	m_speed_pid.SetMode(2);

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			if(t%1500==0){

				m_balance_pid.SetKP( TunableInt::AsFloat(tunableints[0]->GetValue()) );
				m_balance_pid.SetKD( TunableInt::AsFloat(tunableints[1]->GetValue()) );
				m_balance_pid.SetKI( TunableInt::AsFloat(tunableints[2]->GetValue()) );
				m_speed_pid.SetKP( TunableInt::AsFloat(tunableints[3]->GetValue()) );
				m_speed_pid.SetKD( TunableInt::AsFloat(tunableints[4]->GetValue()) );
				m_speed_pid.SetKI( TunableInt::AsFloat(tunableints[5]->GetValue()) );
				m_turn_pid.SetKP( TunableInt::AsFloat(tunableints[6]->GetValue()) );
				m_turn_pid.SetKD( TunableInt::AsFloat(tunableints[7]->GetValue()) );
				speed_input_smoothing.UpdateCurrentOutput(TunableInt::AsFloat(tunableints[8]->GetValue()) );
				if(TunableInt::AsFloat(tunableints[9]->GetValue())==e_stop) eStop();

				/*printf("b_kp: %f\n",b_kp[1]);
				printf("b_kd: %f\n",b_kd[1]);
				printf("b_ki: %f\n",b_ki[1]);
				printf("s_kp: %f\n",s_kp[1]);
				printf("s_kd: %f\n",s_kd[1]);
				printf("s_ki: %f\n",s_ki[1]);
				printf("t_kp: %f\n",t_kp[1]);
				printf("t_kd: %f\n",t_kd[1]);
				printf("SPEED_SETPOINTS: %f\n",SPEED_SETPOINTS[1]);*/
			}

			m_speed_pid.SetSetPoint( speed_input_smoothing.SmoothingOutput() );


			if(t%100==0) {
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 1,
//				libutil::String::Format("%.3f",angle[0]).c_str());
				printf("degree: %f\n", angle[0]);
//				const char* s = libutil::String::Format("%.3f",b_kp[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 0, s);
//				s = libutil::String::Format("%.3f",b_kd[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 1, s);
//				s = libutil::String::Format("%.3f",b_ki[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 2, s);
//				s = libutil::String::Format("%.3f",s_kp[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 3, s);
//				s = libutil::String::Format("%.3f",s_kd[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 4, s);
//				s = libutil::String::Format("%.3f",s_ki[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 5, s);
//				s = libutil::String::Format("%.3f",t_kp[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 6, s);
//				s = libutil::String::Format("%.3f",t_kd[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 7, s);
//				s = libutil::String::Format("%d",SPEED_SETPOINTS[1]).c_str();
//				m_helper.Printline(m_car.GetLcd()->FONT_W * 0, m_car.GetLcd()->FONT_H * 8, s);
			}

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Turn Control Output every 1 ms///
			// TurnControlOutput();


			if(t%2==0){
				mpu6050_update();

			}

			if(t%2==1){

				BalanceControl();

			}

			if(t - pt > 5000) {

				if(t%TURNCONTROLPERIOD==1 && num_finished_row==0){
					ProcessImage();
					num_finished_row+=15;
				}

				if(t%TURNCONTROLPERIOD==3 && num_finished_row==15){
					ProcessImage();
					num_finished_row+=15;
				}

				if(t%TURNCONTROLPERIOD==5 && num_finished_row==30){
					ProcessImage();
					num_finished_row+=15;
				}

				if(t%TURNCONTROLPERIOD==7 && num_finished_row==45){
					ProcessImage();
					num_finished_row=0;
					TurnControl();
				}

			}


			///Speed PID update every 20ms///
			if(t%SPEEDCONTROLPERIOD==0){
//				printf("DT: %d\n", libutil::Clock::TimeDiff(libutil::Clock::Time(),dt));
				SpeedControl();
//				dt = libutil::Clock::Time();
			}
			MoveMotor();

		}


//		if(m_start_button->IsDown()) {
//			autoprint = !autoprint;
//			while(m_start_button->IsDown());
//		}

	}

}

void CameraApp::PidMode()
{
	uint32_t t;
	while(true){
		t = libutil::Clock::Time();
		if(t%1500==0){
//								b_kp[1] = TunableInt::AsFloat(tunableints[0]->GetValue());
//								b_kd[1] = TunableInt::AsFloat(tunableints[1]->GetValue());
//								b_ki[1] = TunableInt::AsFloat(tunableints[2]->GetValue());
//								s_kp[1] = TunableInt::AsFloat(tunableints[3]->GetValue());
//								s_kd[1] = TunableInt::AsFloat(tunableints[4]->GetValue());
//								s_ki[1] = TunableInt::AsFloat(tunableints[5]->GetValue());
//								t_kp[1] = TunableInt::AsFloat(tunableints[6]->GetValue());
//								t_kd[1] = TunableInt::AsFloat(tunableints[7]->GetValue());
//								SPEED_SETPOINTS[1] = TunableInt::AsFloat(tunableints[8]->GetValue());
								/*printf("b_kp: %f\n",b_kp[1]);
								printf("b_kd: %f\n",b_kd[1]);
								printf("b_ki: %f\n",b_ki[1]);
								printf("s_kp: %f\n",s_kp[1]);
								printf("s_kd: %f\n",s_kd[1]);
								printf("s_ki: %f\n",s_ki[1]);
								printf("t_kp: %f\n",t_kp[1]);
								printf("t_kd: %f\n",t_kd[1]);
								printf("SPEED_SETPOINTS: %f\n",SPEED_SETPOINTS[1]);*/
			}

		if(t%1500==0 /*&& autoprint*/) {
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 1,
//					libutil::String::Format("%.3f",angle[0]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 2,
//					libutil::String::Format("%.3f",b_kd[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 3,
//							libutil::String::Format("%.3f",b_ki[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 4,
//							libutil::String::Format("%.3f",s_kp[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 5,
//							libutil::String::Format("%.3f",s_kd[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 6,
//							libutil::String::Format("%.3f",s_ki[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 7,
//							libutil::String::Format("%.3f",t_kp[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 8,
//							libutil::String::Format("%.3f",t_kd[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 9,
//							libutil::String::Format("%d",SPEED_SETPOINTS[1]).c_str());
//			m_helper.Printline(m_car.GetLcd()->FONT_W * 1, m_car.GetLcd()->FONT_H * 0,
//							libutil::String::Format("%f", TunableInt::AsFloat(tunableints[9]->GetValue()) ).c_str());

		}
	}
}

void CameraApp::AccelAndGyroMode()
{
	uint32_t t = 0;

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
//				gpio_set(PTD4, 1);
				mpu6050_update();
//				gpio_set(PTD4, 0);
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
	uint32_t t;
//	m_balance_pid.SetMode(2);
//	m_turn_pid.SetMode(2);
	m_speed_pid.SetMode(2);
	m_speed_pid.SetSetPoint( 40 );

	while (true)
	{


		///System loop - 1ms///

		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();



			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==0) SpeedControl();

			if(t%1000){
				encoder1 = FTM_QUAD_get(FTM1);
				encoder2 = -FTM_QUAD_get(FTM2);
				s = libutil::String::Format("Encoder1: %d",encoder1).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("Encoder2: %d",encoder2).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("Time: %d",libutil::Clock::Time()/1000).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 3, s);
			}
			m_balance_speed[0] = m_balance_speed[1] = 0;
			m_turn_speed[0] = m_turn_speed[1] = 0;

		}

	}

}

void CameraApp::CameraMode()
{
	uint32_t t = 0;

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
	uint32_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Parade");

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			//SPEEDSETPOINT = 720;
			m_balance_pid.SetMode(3);
			m_turn_pid.SetMode(3);
			m_speed_pid.SetMode(3);
			m_speed_pid.SetSetPoint(SPEED_SETPOINTS[2]);
			///Speed Control Output every 1ms///
			SpeedControlOutput();

			/*if(t%1000==0 && autoprint) {
				const char* s = libutil::String::Format("Speed: %04d,%04d",m_control_speed[0],m_control_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("Motor: %04d, %04d",m_total_speed[0], m_total_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("Angle: %02d",(int)m_gyro).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("SSP: %d",m_speed_pid.GetSetPoint()).c_str();
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
	uint32_t t = 0;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Balance Only");

	while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

//				if(t - pt > 5000) {
					m_balance_pid.SetMode(2);
					m_turn_pid.SetMode(2);
					m_speed_pid.SetMode(2);
//				}



					if(t%1500==0){
//						b_kp[1] = TunableInt::AsFloat(tunableints[0]->GetValue());
//						b_kd[1] = TunableInt::AsFloat(tunableints[1]->GetValue());
//						b_ki[1] = TunableInt::AsFloat(tunableints[2]->GetValue());
//						s_kp[1] = TunableInt::AsFloat(tunableints[3]->GetValue());
//						s_kd[1] = TunableInt::AsFloat(tunableints[4]->GetValue());
//						s_ki[1] = TunableInt::AsFloat(tunableints[5]->GetValue());
//						t_kp[1] = TunableInt::AsFloat(tunableints[6]->GetValue());
//						t_kd[1] = TunableInt::AsFloat(tunableints[7]->GetValue());
//						SPEED_SETPOINTS[1] = TunableInt::AsFloat(tunableints[8]->GetValue());
						/*printf("b_kp: %f\n",b_kp[1]);
						printf("b_kd: %f\n",b_kd[1]);
						printf("b_ki: %f\n",b_ki[1]);
						printf("s_kp: %f\n",s_kp[1]);
						printf("s_kd: %f\n",s_kd[1]);
						printf("s_ki: %f\n",s_ki[1]);
						printf("t_kp: %f\n",t_kp[1]);
						printf("t_kd: %f\n",t_kd[1]);
						printf("SPEED_SETPOINTS: %f\n",SPEED_SETPOINTS[1]);*/
					}

				if(t%1500==0 /*&& autoprint*/) {
					const char* s = libutil::String::Format("%06d",m_control_speed[0]).c_str();
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
					s = libutil::String::Format("%06d",m_control_speed[1]).c_str();
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
					s = libutil::String::Format("%06d",m_total_speed[0]).c_str();
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
					s = libutil::String::Format("%06d",m_total_speed[1]).c_str();
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 4, s);
					s = libutil::String::Format("%d",(int)m_gyro).c_str();
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 5, s);
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 6,
							libutil::String::Format("%.3f",b_kp[1]).c_str());
					m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 7,
							libutil::String::Format("%.3f",b_kd[1]).c_str());
				}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				// TurnControlOutput();


				if(t%2==0){
					mpu6050_update();
				}

				if(t%2==1){

					BalanceControl();
				}

//				if(t - pt > 5000) {

					if(t%TURNCONTROLPERIOD==1 && num_finished_row==0){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==3 && num_finished_row==15){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==5 && num_finished_row==30){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==7 && num_finished_row==45){
						ProcessImage();
						num_finished_row=0;
						TurnControl();
					}

//				}


				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0){
	//				printf("DT: %d\n", libutil::Clock::TimeDiff(libutil::Clock::Time(),dt));
					SpeedControl();
	//				dt = libutil::Clock::Time();
				}

				m_control_speed[0] = m_control_speed[0] = m_turn_speed[0] = m_turn_speed[1] = 0;

				MoveMotor();

			}


	//		if(m_start_button->IsDown()) {
	//			autoprint = !autoprint;
	//			while(m_start_button->IsDown());
	//		}

		}
}

void CameraApp::CameraMoveMode()
{
	uint32_t t = 0;

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
				m_balance_pid.SetMode(2);
				m_turn_pid.SetMode(2);
				m_speed_pid.SetMode(2);
				m_speed_pid.SetSetPoint(SPEED_SETPOINTS[1]);
			//}


			if(t%1500==0 && autoprint) {
				/*const char* s = libutil::String::Format("%06d",m_control_speed[0]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%06d",m_control_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%06d",m_total_speed[0]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%06d",m_total_speed[1]).c_str();
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

			//if(t - pt > 5000) {


			if(t%TURNCONTROLPERIOD==1 && num_finished_row==0){
				ProcessImage();
				num_finished_row+=20;
			}

			if(t%TURNCONTROLPERIOD==3 && num_finished_row==20){
				ProcessImage();
				num_finished_row+=20;
			}

			if(t%TURNCONTROLPERIOD==5 && num_finished_row==40){
				ProcessImage();
				num_finished_row=0;
//				TurnControl();
			}

			//}

			///Speed PID update every 100ms///
			if(t%SPEEDCONTROLPERIOD==1){ SpeedControl(); }
			m_balance_speed[0] = m_balance_speed[1] = 0;
			m_control_speed[0] = m_control_speed[1] = 0;
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
	uint32_t t = 0;
	uint32_t dt = 0;
	uint32_t pt = libutil::Clock::Time();
	bool autoprint = false;

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Balance & Speed");

	while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

//				if(t - pt > 5000) {
					m_balance_pid.SetMode(2);
					m_turn_pid.SetMode(2);
					m_speed_pid.SetMode(2);
					m_speed_pid.SetSetPoint(SPEED_SETPOINTS[1]);
//				}

				if(t%1500==0){
/*					b_kp[1] = TunableInt::AsFloat(tunableints[0]->GetValue());
					b_kd[1] = TunableInt::AsFloat(tunableints[1]->GetValue());
					b_ki[1] = TunableInt::AsFloat(tunableints[2]->GetValue());
					s_kp[1] = TunableInt::AsFloat(tunableints[3]->GetValue());
					s_kd[1] = TunableInt::AsFloat(tunableints[4]->GetValue());
					s_ki[1] = TunableInt::AsFloat(tunableints[5]->GetValue());
					t_kp[1] = TunableInt::AsFloat(tunableints[6]->GetValue());
					t_kd[1] = TunableInt::AsFloat(tunableints[7]->GetValue());*/
					SPEED_SETPOINTS[1] = TunableInt::AsFloat(tunableints[8]->GetValue());
					/*printf("b_kp: %f\n",b_kp[1]);
					printf("b_kd: %f\n",b_kd[1]);
					printf("b_ki: %f\n",b_ki[1]);
					printf("s_kp: %f\n",s_kp[1]);
					printf("s_kd: %f\n",s_kd[1]);
					printf("s_ki: %f\n",s_ki[1]);
					printf("t_kp: %f\n",t_kp[1]);
					printf("t_kd: %f\n",t_kd[1]);
					printf("SPEED_SETPOINTS: %f\n",SPEED_SETPOINTS[1]);*/
				}


	//			if(t%1500==0 && autoprint) {
	//				const char* s = libutil::String::Format("%06d",m_control_speed[0]).c_str();
	//				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
	//				s = libutil::String::Format("%06d",m_control_speed[1]).c_str();
	//				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
	//				s = libutil::String::Format("%06d",m_total_speed[0]).c_str();
	//				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
	//				s = libutil::String::Format("%06d",m_total_speed[1]).c_str();
	//				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 4, s);
	//				s = libutil::String::Format("%d",(int)m_gyro).c_str();
	//				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 5, s);
	//			}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				// TurnControlOutput();


				if(t%2==0){
					mpu6050_update();
				}

				if(t%2==1){

					BalanceControl();
				}

				if(t - pt > 5000) {

					if(t%TURNCONTROLPERIOD==1 && num_finished_row==0){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==3 && num_finished_row==15){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==5 && num_finished_row==30){
						ProcessImage();
						num_finished_row+=15;
					}

					if(t%TURNCONTROLPERIOD==7 && num_finished_row==45){
						ProcessImage();
						num_finished_row=0;
						TurnControl();
					}

				}


				m_turn_speed[0] = m_turn_speed[1] = 0;

				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0){
//					printf("t:%d \t dt:%d \t DT: %d\n", t, dt, libutil::Clock::TimeDiff(t,dt));
					SpeedControl();
//					dt = t;
				}
				MoveMotor();

			}


	//		if(m_start_button->IsDown()) {
	//			autoprint = !autoprint;
	//			while(m_start_button->IsDown());
	//		}

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
	uint32_t t = 0;

	bool autoprint = false;

	libsc::Button* m_start_button = m_car.GetButton();

	m_helper.Printline(m_car.GetLcd()->FONT_H * 0, "Speed");

	while (true)
	{
		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();
			m_balance_speed[0] = m_balance_speed[0] = m_turn_speed[0] = m_turn_speed[1] = 0;
			//SPEEDSETPOINT = 0;

			if(t%1000==0 && autoprint) {
				const char* s = libutil::String::Format("Speed: %d,%d",m_control_speed[0],m_control_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("Motor: %d, %d",m_total_speed[0], m_total_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_H * 2, s);

				s = libutil::String::Format("SSP: %d", m_speed_pid.GetSetPoint()).c_str();
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
	uint32_t t = 0;
	uint32_t pt = 0;

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
				m_balance_pid.SetMode(2);
				m_turn_pid.SetMode(2);
				m_speed_pid.SetMode(2);
				m_speed_pid.SetSetPoint(SPEED_SETPOINTS[1]);
			}


			if(t%1500==0 && autoprint) {
				/*const char* s = libutil::String::Format("%06d",m_control_speed[0]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%06d",m_control_speed[1]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%06d",m_total_speed[0]).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 8, m_car.GetLcd()->FONT_H * 3, s);
				s = libutil::String::Format("%06d",m_total_speed[1]).c_str();
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
	uint32_t t = 0;
	uint32_t pt = 0;

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
