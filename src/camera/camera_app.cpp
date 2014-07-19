/*
 * camera_app.cpp
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>
#include <hw_common.h>
#include <vectors.h>
#include <MK60_pit.h>
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
#include "math_tools.h"
#include "upstand_signal.h"

using namespace libutil;

namespace
{

volatile bool g_balance_control_flag = false;

}

__ISR void Pit4Handler(void)
{
	g_balance_control_flag = true;
}


namespace camera
{
float b_kp[3] = {1600.0f, 1600.0f, 1600.0f};
float b_ki[3] = {12.0f, 12.0f, 12.0f};
float b_kd[3] = {10.0f, 10.0f, 10.0f};

//float b_kp[3] = {800.0f, 800.0f, 800.0f};
//float b_ki[3] = {0.0, 0.0, 0.0};
//float b_kd[3] = {160.0f, 160.0f, 160.0f};

float SPEED_SETPOINTS[3] = {0.0, 0.0, 0.0};

float s_kp[3] = {50.0, 0.0, 0.0};
float s_ki[3] = {10.1, 0.0, 0.0};
float s_kd[3] = {0.0, 0.0, 0.0};

float t_kp[3] = {0.0, 0.0, 0.0};
float t_ki[3] = {0.0, 0.0, 0.0};
float t_kd[3] = {0.0, 0.0, 0.0};

float BALANCE_SETPOINT = 48.0f;

float feedback_angle;

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
//	m_balance_pid(BALANCE_SETPOINT, b_kp, b_ki, b_kd, 3, 1),
	m_balance_pid(BALANCE_SETPOINT, b_kp, b_ki, 0, 3, 1),
	m_balance_derivative_pid(0, b_kd, 0, 0, 3, 1),
	speed_smoothing(SPEEDCONTROLPERIOD),
	speed_input_smoothing(2000),
	turn_smoothing(TURNCONTROLPERIOD),
	m_gyro(0),
	m_encoder_2(0),
	encoder_total(0),
	m_count(0),
	num_finished_row(0),
	src(NULL),
	e_stop(0),
	start_row(0),
	end_row(0),
	stopped(false),
	t(libutil::Clock::Time()),
	num_finished_laps(0),
	m_acc(0)
{
	printf("Voltage: %f\r\n", m_car.GetVolt());

	gpio_init(PTB22, GPO, 0);

	libutil::Clock::Init();
//	kalman_filter_init(&m_gyro_kf[1], 0.0012, 0.012, 0, 1);
//	kalman_filter_init(&m_gyro_kf[2], 0.0012, 0.012, 0, 1);
//	kalman_filter_init(&m_acc_kf, 0.0005, 0.05, 0, 1);
	g_m_gyro = &m_gyro;
	mpu6050_init();
	mpu6050_update(&m_gyro);

	tunableints[0] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("bkp", TunableInt::REAL,
			TunableInt::AsUnsigned(b_kp[0]));
	tunableints[1] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("bkd", TunableInt::REAL,
				TunableInt::AsUnsigned(b_kd[0]));
	tunableints[2] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("bki", TunableInt::REAL,
					TunableInt::AsUnsigned(b_ki[0]));
	tunableints[3] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("skp", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kp[0]));
	tunableints[4] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("skd", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kd[0]));
	tunableints[5] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("ski", TunableInt::REAL,
						TunableInt::AsUnsigned(s_ki[0]));
	tunableints[6] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("tkp", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kp[0]));
	tunableints[7] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("tkd", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kd[0]));
	tunableints[8] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("speed", TunableInt::INTEGER,
						TunableInt::AsSigned(SPEED_SETPOINTS[1]));
	tunableints[9] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("turn_multiplier", TunableInt::INTEGER,
						TunableInt::AsSigned(10));
	tunableints[10] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("speed--", TunableInt::REAL,
						TunableInt::AsUnsigned(0.0f));
	tunableints[11] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("estop", TunableInt::INTEGER,
						TunableInt::AsSigned(e_stop));
	tunableints[12] = TunableIntManager<13>::GetInstance(m_car.GetUart())->Register("BalanceSetPt", TunableInt::REAL,
						TunableInt::AsSigned(BALANCE_SETPOINT));

	TunableIntManager<13>::GetInstance(m_car.GetUart())->Start();

	__g_hard_fault_handler = HardFaultHandler;
	m_instance = this;
}

CameraApp::~CameraApp()
{

}

void CameraApp::eStop(){
	printf("estop\n");
	m_car.MoveMotor(0, 0);
	m_car.MoveMotor(1, 0);


	m_speed_pid.ResetError();
	m_speed_pid.SetSetPoint(0);
	m_balance_pid.ResetError();

	m_turn_pid.ResetError();
	m_speed_pid.SetSetPoint(0);
	speed_input_smoothing.UpdateCurrentOutput(0);

	while(m_car.GetJoystick()->GetState() != libsc::Joystick::SELECT);

}

void CameraApp::BalanceControl()
{
	m_balance_pid.SetKP( TunableInt::AsFloat(tunableints[0]->GetValue()) );
//	m_balance_pid.SetKD( TunableInt::AsFloat(tunableints[1]->GetValue()) );
	m_balance_pid.SetKI( TunableInt::AsFloat(tunableints[2]->GetValue()) );

	m_balance_derivative_pid.SetKP( TunableInt::AsFloat(tunableints[1]->GetValue()) );
	///Get values from gyro///

	m_car.AccelRefresh();
	m_acc = m_car.GetRawAngle();
	window_update(&acc_moving, m_acc);

	if(moving_struct_get_window_count(&acc_moving)==moving_struct_get_window_size(&acc_moving)-1){
		float value = avg(moving_struct_get_window(&acc_moving), moving_struct_get_window_size(&acc_moving));
		kalman_filtering(&m_gyro_kf[0], &feedback_angle, &m_gyro, &value, 1);
	}
	else{
		feedback_angle = m_gyro;
	}
	kalman_filtering(&m_gyro_kf[0], &feedback_angle, &m_gyro, &m_acc, 1);

	fixedangle = 90 + feedback_angle;

//	m_balance_pid.SetSetPoint( TunableInt::AsFloat( tunableints[12]->GetValue()) );
	m_balance_pid.SetSetPoint( BALANCE_SETPOINT );

	m_balance_derivative_pid.SetSetPoint(0);

//	m_balance_pid.UpdateCurrentError(gl_angle);
	m_balance_pid.UpdateCurrentError(fixedangle);
	m_balance_derivative_pid.UpdateCurrentError(getOmega());

//	m_balance_speed[0] = m_balance_speed[1] = ( m_balance_pid.Proportional() + m_balance_pid.Derivative() + m_balance_pid.Integral() + m_balance_derivative_pid.Proportional() );
	m_balance_speed[0] = m_balance_speed[1] = ( m_balance_pid.Proportional() + m_balance_pid.Integral() + m_balance_derivative_pid.Proportional() );

	m_balance_pid.UpdatePreviousError();
	m_balance_derivative_pid.UpdatePreviousError();
}


void CameraApp::SpeedControl(){
#ifndef LIBSC_USE_K60_ENCODERS
	return;
#endif

	m_speed_pid.SetKP( TunableInt::AsFloat(tunableints[3]->GetValue()) );
	m_speed_pid.SetKD( TunableInt::AsFloat(tunableints[4]->GetValue()) );
	m_speed_pid.SetKI( TunableInt::AsFloat(tunableints[5]->GetValue()) );

	int32_t encoder1 = FTM_QUAD_get(FTM1);
	int32_t encoder2 = -FTM_QUAD_get(FTM2);

	m_encoder_2 = (encoder1 + encoder2)/2;
	encoder_total += m_encoder_2;

	FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);

	m_speed_pid.UpdateCurrentError( m_encoder_2 );

	speed_smoothing.UpdateCurrentOutput( (int32_t) (m_speed_pid.Proportional() + m_speed_pid.Integral() + m_speed_pid.Derivative()) );

	m_speed_pid.UpdatePreviousError();
}


void CameraApp::SpeedControlOutput(){
    m_control_speed[0] = m_control_speed[1] = (int32_t) speed_smoothing.SmoothingOutput();
}

bool CameraApp::isDestination(int x, int y)
{
	bool total = false;
	bool left=false, middle=false, right=false;
    int startx = 10;
	for(int y1=y;y1<=y+2;y1++){
		startx = x;
		left = m_helper.RangeIsColor(startx, startx+20, y1, BLACK_BYTE, src);
		startx+=21;
		middle = m_helper.RangeIsColor(startx, startx+18, y1, WHITE_BYTE, src);
		startx+=19;
		right = m_helper.RangeIsColor(startx, startx+20, y1, BLACK_BYTE, src);
//		printf("%d \t %d \t %d\n", left, middle, right);
		if(left && middle && right) total = true;

		if(total) break;
	}
	if(total) printf("ENDLINE\n");


	return total;
}

void CameraApp::ProcessImage(){
	start_row = num_finished_row;
	end_row = num_finished_row+15;
	bool ready = m_car.GetCamera()->IsImageReady();
	static bool locked = false;
	static int32_t temp=0;

	if(num_finished_row==0) {
		if(ready) {
			src = m_car.GetCamera()->LockBuffer();
			locked = true;
			white_dot[1] = white_dot[0] = 0;
		}
		else {
			locked = false;
			return;
		}
	}

	if(locked){
		for(int y=start_row; y<end_row; y++)
		{
			if((encoder_total>=28*7200))
			{
				for(int x=2; x<=12; x+=2){
					if(y==47 && isDestination(x,y)){
						num_finished_laps++;
						temp = encoder_total;
						break;
					}
				}

				if(num_finished_laps>=0 && encoder_total>=temp+7200) {
					speed_smoothing.SetOutputPeriod(3000);
					m_speed_pid.SetSetPoint( 0 );
					stopped=true;
				}

			}

			for(int x=0; x<CAM_W; x++)
			{
				if(m_helper.GetPixel(src, x, y) == WHITE_BYTE)
				{
					x>=(CAM_W/2) ? white_dot[1]++ : white_dot[0]++;

				}
			}
			if(num_finished_row==0){
				white_dot_z[1]=white_dot[1];
				white_dot_z[0]=white_dot[0];
			}
		}
	}



}

void CameraApp::TurnControl(){
	int32_t error = white_dot[0] - white_dot[1];

	if(error==0){
		error = turn_error[1] - turn_error[0] + turn_error[1];
	}

	turn_error[0]= turn_error[1];
	turn_error[1]= error;


	m_turn_pid.UpdateCurrentError(error);


	m_turn_pid.SetKP( TunableInt::AsFloat( tunableints[6]->GetValue()) );
	m_turn_pid.SetKD( TunableInt::AsFloat( tunableints[7]->GetValue()) );

	int32_t degree = (int32_t) round(
		m_turn_pid.Proportional() +   m_turn_pid.Derivative()	);

	m_turn_pid.UpdatePreviousError();

	turn_smoothing.UpdateCurrentOutput( -1 * degree * 10 );

	m_car.GetCamera()->UnlockBuffer();
}

void CameraApp::TurnControlOutput(){
	m_turn_speed[0] = turn_smoothing.SmoothingOutput();
	m_turn_speed[1] = -m_turn_speed[0];
}

void CameraApp::MoveMotor(){

	m_total_speed[0] = m_balance_speed[0] - m_control_speed[0] + m_turn_speed[0];
	m_total_speed[1] = m_balance_speed[1] - m_control_speed[1] + m_turn_speed[1];

	m_dir[0] = m_total_speed[0] > 0 ? true : false;
	m_dir[1] = m_total_speed[1] > 0 ? true : false;

	m_car.MotorDir(0, !m_dir[0]); 			////Right Motor - True Backward  -  False Forward
	m_car.MoveMotor(0,(uint16_t) m_helper.abs(m_total_speed[0]));

	m_car.MotorDir(1, m_dir[1]);			////Left Motor - False Backward  -  True Forward
	m_car.MoveMotor(1,(uint16_t) m_helper.abs(m_total_speed[1]));
}

void CameraApp::AutoMode()
{
		while (true)
		{

			if(e_stop != TunableInt::AsFloat(tunableints[11]->GetValue()) ){
				e_stop = TunableInt::AsFloat(tunableints[11]->GetValue());
				eStop();
			}

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(), t)>0){
				t = libutil::Clock::Time();
				if(!stopped && t%2000==0){
					speed_input_smoothing.UpdateCurrentOutput( TunableInt::AsFloat(tunableints[8]->GetValue()) );
				}

				if(!stopped) {
					m_speed_pid.SetSetPoint( speed_input_smoothing.SmoothingOutput() );
				}


				if(t%2==0){
					mpu6050_update(&m_gyro);
					//kalman_filter();
					BalanceControl();
				}


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
					TurnControl();
					num_finished_row=0;
				}

				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0){
					SpeedControl();
				}


				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				TurnControlOutput();



				if(t%5==0) MoveMotor();

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

	while (true)
	{

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();

			///Update Gyro every 2ms///
			if(t % 2 == 0)	{
				mpu6050_update(&m_gyro);
				BalanceControl();
			}


			if(t%200==0) {

				const char* s = libutil::String::Format("%.2f",fixedangle).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 1, s);
				s = libutil::String::Format("%.3f",m_car.GetRawAngle()).c_str();
				m_helper.Printline(m_car.GetLcd()->FONT_W * 7, m_car.GetLcd()->FONT_H * 2, s);
				s = libutil::String::Format("%.3f", angle[0]).c_str();
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
	uint32_t encoder1, encoder2;
//	m_balance_pid.SetMode(2);
//	m_turn_pid.SetMode(2);
//	m_speed_pid.SetMode(2);
//	m_speed_pid.SetSetPoint( 40 );

	while (true)
	{


		///System loop - 1ms///

		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
			t = libutil::Clock::Time();


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
			if(m_car.GetCamera()->IsImageReady())
			{
				src = m_car.GetCamera()->LockBuffer();
				m_helper.PrintCam();
				for(int x=5; x<=15; x++) isDestination(x,50);
				m_car.GetCamera()->UnlockBuffer();
			}
		}

	}
}


void CameraApp::BalanceOnlyMode()
{
	uint32_t t = 0, set_time = 0;
		uint32_t dt = 0;
		uint32_t pt = libutil::Clock::Time();

		libsc::Joystick* m_joystick = m_car.GetJoystick();

	//	m_balance_pid.SetMode(2);
	//	m_turn_pid.SetMode(2);
	//	m_speed_pid.SetMode(2);

		while (true)
		{

			if(e_stop != TunableInt::AsFloat(tunableints[11]->GetValue()) ){
				e_stop = TunableInt::AsFloat(tunableints[11]->GetValue());
				eStop();
			}

			if(m_joystick->GetState()==libsc::Joystick::DOWN){
				printf("startcar\n");
			}

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(), t)>0){
				t = libutil::Clock::Time();

				if(!stopped && t%2000==0 && m_speed_pid.GetSetPoint() < 1){
					speed_input_smoothing.UpdateCurrentOutput( TunableInt::AsFloat(tunableints[8]->GetValue()) );
				}

				if(!stopped) {
					m_speed_pid.SetSetPoint( speed_input_smoothing.SmoothingOutput() );
				}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				TurnControlOutput();



				if(t%2==0){
					mpu6050_update(&m_gyro);
//					kalman_filter();
					BalanceControl();
				}


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
//					TurnControl();
					num_finished_row=0;
				}

				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0){
//					SpeedControl();
				}
				if(t%5==0) MoveMotor();

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

					SPEED_SETPOINTS[1] = TunableInt::AsFloat(tunableints[8]->GetValue());

				}



				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				// TurnControlOutput();


				if(t%2==0){
					mpu6050_update(&m_gyro);
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
//					printf("t:%d \t dt:%d \t DT: %d\r\n", t, dt, libutil::Clock::TimeDiff(t,dt));
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
	m_total_speed[0] = m_total_speed[1] = 0;
	int32_t encoder1, encoder2;
	libsc::Joystick* m_joystick = m_car.GetJoystick();
	int32_t t, pt;
	int count = 0;
	while(1){
		t = libutil::Clock::Time();
		if(libutil::Clock::TimeDiff(t,pt)>=20){
//			if(m_joystick->GetState() == libsc::Joystick::DOWN && m_total_speed[0]-100>=0){
//				m_total_speed[0] -= 100;
//				m_total_speed[1] -= 100;
//				while(m_joystick->GetState() == libsc::Joystick::DOWN);
//			}
//			if(m_joystick->GetState() == libsc::Joystick::UP && m_total_speed[0]+100<=10000){
				if(count%50==0){
					m_total_speed[0] += 100;
					m_total_speed[1] += 100;
				}

//				while(m_joystick->GetState() == libsc::Joystick::UP);
//			}

			m_dir[0] = m_total_speed[0] > 0 ? true : false;
			m_dir[1] = m_total_speed[1] > 0 ? true : false;

			m_car.MotorDir(0, !m_dir[0]); 			////Right Motor - True Backward  -  False Forward
			m_car.MoveMotor(0,(uint16_t) m_helper.abs(m_total_speed[0]/0.793206741));

			m_car.MotorDir(1, m_dir[1]);			////Left Motor - False Backward  -  True Forward
			m_car.MoveMotor(1,(uint16_t) m_helper.abs(m_total_speed[1]));

			encoder1 = FTM_QUAD_get(FTM1);
			encoder2 = -FTM_QUAD_get(FTM2);

			m_encoder_2 = (encoder1 + encoder2)/2;

			FTM_QUAD_clean(FTM1);
			FTM_QUAD_clean(FTM2);
			if(count%50==0) printf("%d \t %d \t %d \t  %f\n",m_total_speed[0], encoder2, encoder1, ((float)encoder2)/encoder1);
			count++;
			pt = t;
		}

	}
}

void CameraApp::Run()
{
	m_car.GetLcd()->Clear(WHITE);

	char* title = "Mode:";
	char* choices[11] = {
		"Auto",				//1
		"Accel & Gyro",		//2
		"Encoder",			//3
		"Camera",			//4
		"Balance Only",		//5
		"Balance & Speed",	//6
		"Move motor",		//7
		".",
		"..",
		"...",
		"...."
	};

	LcdMenu lcdmenu(&m_car, title, choices, 11);
	lcdmenu.CreateMenu();
	lcdmenu.WaitForSelection();

	moving_struct_init(&acc_moving, buffer, 100);
	float value[2] = {0.00001f, 0.705495694f};
	for(int i=0; i<100; i++) {
		m_car.AccelRefresh();
		m_acc = m_car.GetRawAngle();
		window_update(&acc_moving, m_acc);
	}
	feedback_angle = m_gyro = m_acc = avg(moving_struct_get_window(&acc_moving), moving_struct_get_window_size(&acc_moving));

	kalman_filter_init(&m_gyro_kf[0], 0.01f, value, m_acc, 1);
	//	kalman_filter_init(&m_gyro_kf[0], 0.025f, 0.1f, m_acc, 1);

	moving_struct_init(&acc_moving, buffer, 10);

	m_car.GetLcd()->Clear(WHITE);

	switch(lcdmenu.GetSelectedChoice()){
		case 1:
			AutoMode();
			break;
		case 2:
			AccelAndGyroMode();
			break;
		case 3:
			EncoderMode();
			break;
		case 4:
			CameraMode();
			break;
		case 5:
			BalanceOnlyMode();
			break;
		case 6:
			BalanceAndSpeedMode();
			break;
		case 7:
			MoveMotorMode();
			break;
		default:
			break;
	}
}

void CameraApp::HardFaultHandler()
{
	m_instance->m_car.MoveMotor(0, 0);
	m_instance->m_car.MoveMotor(1, 0);
	bool flag = true;
	while (true)
	{
		m_instance->m_car.SwitchLed(0, flag);
		DELAY_MS(100);
		m_instance->m_car.SwitchLed(1, flag);
		DELAY_MS(100);
		m_instance->m_car.SwitchLed(2, flag);
		DELAY_MS(100);
		m_instance->m_car.SwitchLed(3, flag);
		DELAY_MS(100);
		flag ^= true;
	}
}

void CameraApp::PitIndicator()
{
//	error_prints();
//	static bool flag = true;
//	m_instance->m_car.SwitchLed(2, !flag);
//	m_instance->m_car.SwitchLed(3, flag);
//	flag ^= true;
//	PIT_Flag_Clear(PIT1);
}

CameraApp *CameraApp::m_instance = nullptr;

}
