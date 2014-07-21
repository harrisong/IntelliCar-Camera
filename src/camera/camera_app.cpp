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
#include <libutil/misc.h>
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

using namespace libutil;


namespace camera
{

float b_kp[3] = {1200.0, 1200.0, 1200.0};
float b_ki[3] = {0.0052, 0.0052, 0.0052};
float b_kd[3] = {30000.0, 30000.0, 30000.0};

int16_t SPEED_SETPOINTS[3] = {100.0f, 150.0f, 200.0f};

float s_kp[3] = {50.0f, 50.0f, 50.0f};
float s_ki[3] = {0.0f, 0.0f, 0.0f};
float s_kd[3] = {0.001f, 0.001f, 0.001f};

float t_kp[3] = {0.2f, 0.2f, 0.2f};
float t_ki[3] = {0.0f, 0.0f, 0.0f};
float t_kd[3] = {5.0f, 5.0f, 5.0f};

float t_kp_s[3] = {0.45, 0.45, 0.45};
float t_kd_s[3] = {5.0, 5.0, 5.0};

float s_s_kp[3] = {15.0, 15.0, 15.0};
float s_s_ki[3] = {0.3, 0.3, 0.3};

__ISR void CameraApp::Pit3Handler()
{
	float acc;
	mpu6050_update();
	for(int i = 0; i < moving_struct_get_window_size(&m_instance->acc_moving); i++){
		m_instance->m_car.AccelRefresh();
		acc = m_instance->m_car.GetRawAngle();
		window_update(&m_instance->acc_moving, acc);
	}
	acc = avg(moving_struct_get_window(&m_instance->acc_moving), moving_struct_get_window_size(&m_instance->acc_moving));
	float a = getAngle();

	kalman_filtering(&m_gyro_kf[0], &m_instance->m_gyro, &a, &acc, 1);

	PIT_Flag_Clear(PIT3);
}


CameraApp::CameraApp():
	m_helper(&m_car),
	m_balance_speed{0, 0},
	m_control_speed{0, 0},
	m_turn_speed{0, 0},
	m_total_speed{0, 0},
	m_dir{false, false},
	white_dot{0, 0},
	m_speed_pid(0, 8000, s_kp, s_ki, s_kd, 3, 1), //SETPOINT, kp array, ki array, kd array, max_mode, initital mode (from 1 to max_mode);
	m_turn_pid(TURN_SETPOINT, 8000, t_kp, t_ki, t_kd, 3, 1),
	m_balance_pid(BALANCE_SETPOINT, 8000, b_kp, b_ki, b_kd, 3, 1),
	speed_smoothing(SPEEDCONTROLPERIOD),
	speed_input_smoothing(SPEEDINPUTPERIOD),
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
	mode_chosen(-1)
{
	m_instance = this;
	printf("Voltage: %f\r\n", m_car.GetVolt());

	gpio_init(PTB22, GPO, 0);

	libutil::Clock::Init();

	mpu6050_init();

	moving_struct_init(&acc_moving, buffer, 100);
	float acc = 0;
	for(int i = 0; i < moving_struct_get_window_size(&acc_moving); i++){
		m_car.AccelRefresh();
		acc = m_car.GetRawAngle();
		window_update(&acc_moving, acc);
	}

	acc = avg(moving_struct_get_window(&acc_moving), moving_struct_get_window_size(&acc_moving));

	float value[2] = {0.0005f,0.7f};
	kalman_filter_init(&m_gyro_kf[0], 0.00001f, value, acc, 1);
	m_gyro = angle[0] = acc;

	moving_struct_init(&acc_moving, buffer, 10);

	pit_init_ms(PIT3, 2);
	SetIsr(PIT3_VECTORn, Pit3Handler);
	EnableIsr(PIT3_VECTORn);

	gpio_set(PTB22, 1);
	DELAY_MS(100);
	gpio_set(PTB22, 0);

	m_car.GetLcd()->Clear(WHITE);

	tunableints[0] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("bkp", TunableInt::REAL,
			TunableInt::AsUnsigned(b_kp[0]));
	tunableints[1] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("bkd", TunableInt::REAL,
				TunableInt::AsUnsigned(b_kd[0]));
	tunableints[2] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("bki", TunableInt::REAL,
					TunableInt::AsUnsigned(b_ki[0]));
	tunableints[3] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("skp", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kp[0]));
	tunableints[4] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("skd", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kd[0]));
	tunableints[5] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("ski", TunableInt::REAL,
						TunableInt::AsUnsigned(s_ki[0]));
	tunableints[6] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("tkp", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kp[0]));
	tunableints[7] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("tkd", TunableInt::REAL,
						TunableInt::AsUnsigned(t_kd[0]));
	tunableints[8] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("speed", TunableInt::INTEGER,
						TunableInt::AsSigned(SPEED_SETPOINTS[0]));
	tunableints[9] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("turn_multiplier", TunableInt::INTEGER,
						TunableInt::AsSigned(15));
	tunableints[10] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("threshold", TunableInt::INTEGER,
						TunableInt::AsSigned(1));
	tunableints[11] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("tkp_s", TunableInt::REAL,
						TunableInt::AsUnsigned(0.22f));
	tunableints[12] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("tkd_s", TunableInt::REAL,
						TunableInt::AsUnsigned(0.30f));
	tunableints[13] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("speed++", TunableInt::REAL,
						TunableInt::AsUnsigned(0.0f));
	tunableints[14] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("speed--", TunableInt::REAL,
						TunableInt::AsUnsigned(0.0f));
	tunableints[15] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("speed--_kp", TunableInt::REAL,
						TunableInt::AsUnsigned(1.5f));
	tunableints[16] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("s_skp", TunableInt::REAL,
						TunableInt::AsUnsigned(s_kp[0]));
	tunableints[17] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("s_ski", TunableInt::REAL,
						TunableInt::AsUnsigned(s_ki[0]));
	tunableints[18] = TunableIntManager<19>::GetInstance(m_car.GetUart())->Register("estop", TunableInt::INTEGER,
						TunableInt::AsSigned(0));
	TunableIntManager<19>::GetInstance(m_car.GetUart())->Start();

	__g_hard_fault_handler = HardFaultHandler;
}

CameraApp::~CameraApp()
{

}

void CameraApp::eStop(){
	m_car.MoveMotor(0, 0);
	m_car.MoveMotor(1, 0);

	m_speed_pid.ResetError();
	m_speed_pid.SetSetPoint(0);
	m_balance_pid.ResetError();
	m_turn_pid.ResetError();
	m_speed_pid.SetSetPoint(0);
	speed_input_smoothing.UpdateCurrentOutput(0);

	while(m_car.GetJoystick()->GetState() != libsc::Joystick::SELECT);

	e_stop++;
}

void CameraApp::BalanceControl()
{

	static bool is_state_changed = false;
	if(m_speed_pid.GetSetPoint() < 1 && !is_state_changed) {
//		m_balance_pid.SetSetPoint(40);
		is_state_changed = true;
	}

	if(mode_chosen==-1){
		m_balance_pid.SetKP( TunableInt::AsFloat(tunableints[0]->GetValue()) );
		m_balance_pid.SetKD( TunableInt::AsFloat(tunableints[1]->GetValue()) );
		m_balance_pid.SetKI( TunableInt::AsFloat(tunableints[2]->GetValue()) );
	}else{
		m_balance_pid.SetKP( b_kp[mode_chosen] );
		m_balance_pid.SetKD( b_kd[mode_chosen] );
		m_balance_pid.SetKI( b_ki[mode_chosen] );
	}
	///Get values from gyro///
	m_car.AccelRefresh();

	float acc = m_car.GetRawAngle();
	float a = getAngle();
	kalman_filtering(&m_gyro_kf[0], &m_gyro, &a, &acc, 1);


//	m_gyro = 1/TIMECONST * acc + (1- 1/TIMECONST) * angle[0];

	m_balance_pid.UpdateCurrentError(m_gyro);

	m_balance_speed[0] = m_balance_speed[1] = ((int32_t) ( m_balance_pid.Proportional() + m_balance_pid.Integral() + m_balance_pid.Derivative() ));

	m_balance_pid.UpdatePreviousError();
}


void CameraApp::SpeedControl(){
#ifndef LIBSC_USE_K60_ENCODERS
	return;
#endif

	int32_t encoder1 = FTM_QUAD_get(FTM1);
	int32_t encoder2 = -FTM_QUAD_get(FTM2);

	m_encoder_2 = (encoder1 + encoder2)/2;

	encoder_total += m_encoder_2;


	FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);

	if(mode_chosen==-1){
		m_speed_pid.SetKP( TunableInt::AsFloat(tunableints[3]->GetValue()) );
		m_speed_pid.SetKI( TunableInt::AsFloat(tunableints[4]->GetValue()) );
		m_speed_pid.SetKD( TunableInt::AsFloat(tunableints[5]->GetValue()) );
	}else{
		m_speed_pid.SetKP( s_kp[mode_chosen] );
		m_speed_pid.SetKI( s_ki[mode_chosen] );
		m_speed_pid.SetKD( s_kd[mode_chosen] );
	}

//	if(m_encoder_2 <= ((int32_t) TunableInt::AsFloat(tunableints[8]->GetValue()))
//		|| m_speed_pid.GetSetPoint() < 1)
//	{
//		m_speed_pid.SetKP( TunableInt::AsFloat(tunableints[16]->GetValue()) );
//		m_speed_pid.SetKI( TunableInt::AsFloat(tunableints[17]->GetValue()) );
//	} else {
//		m_speed_pid.SetKP( TunableInt::AsFloat(tunableints[3]->GetValue()) );
//		m_speed_pid.SetKI( TunableInt::AsFloat(tunableints[5]->GetValue()) );
//	}

	m_speed_pid.UpdateCurrentError( m_encoder_2 );

	speed_smoothing.UpdateCurrentOutput( (m_speed_pid.Proportional() + m_speed_pid.Integral() + m_speed_pid.Derivative()) );

	m_speed_pid.UpdatePreviousError();
}


void CameraApp::SpeedControlOutput(){

    m_control_speed[0] = m_control_speed[1] = speed_smoothing.SmoothingOutput();
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

	if(num_finished_row==0) {
		if(ready) {
			src = m_car.GetCamera()->LockBuffer();
			locked = true;
			white_dot[1] = white_dot[0] = 0;
		} else {
			locked = false;
		}
	}

	if(locked){
		for(int y=start_row; y<end_row; y++)
		{
			/*
			if((encoder_total>=28*7200))
			{
				for(int x=2; x<=10; x+=2){
					if(y==47 && (stopped || isDestination(x,y))){
						num_finished_laps++;
						static int32_t temp = encoder_total;
						encoder_total = 0;
						if(num_finished_laps>=4 && encoder_total>=temp+7200) {
							stopped = true;
							speed_smoothing.SetOutputPeriod(3000);
							m_speed_pid.SetSetPoint( 0 );
						}
						break;
					}
				}

			}
			*/

			for(int x=0; x<CAM_W; x++)
			{
				if(m_helper.GetPixel(src, x, y) == WHITE_BYTE)
				{
					x>=(CAM_W/2) ? white_dot[1]++ : white_dot[0]++;
				}
			}
		}
	}



}

void CameraApp::TurnControl(){
	int32_t error = white_dot[0] - white_dot[1];

	//printf("dot: %ld, %ld\n", white_dot[0], white_dot[1]);

	if(error==0){
		error = turn_error[1] - turn_error[0] + turn_error[1];
	}

	turn_error[0] = turn_error[1];
	turn_error[1] = error;

	m_turn_pid.UpdateCurrentError(error);

	if(mode_chosen==-1){
		m_turn_pid.SetKP( TunableInt::AsFloat( tunableints[6]->GetValue()) );
		m_turn_pid.SetKD( TunableInt::AsFloat( tunableints[7]->GetValue()) );
	}else{
		m_turn_pid.SetKP( t_kp[mode_chosen] );
		m_turn_pid.SetKD( t_kd[mode_chosen] );
	}

//	int32_t s_error  = (int32_t) (abs(error) / 100.0f);
//	if(s_error <= (int32_t) TunableInt::AsFloat(tunableints[10]->GetValue()))
//	{
//		gpio_set(PTB22, 0);
//		m_turn_pid.SetKP( TunableInt::AsFloat( tunableints[11]->GetValue()) );
//		m_turn_pid.SetKD( TunableInt::AsFloat( tunableints[12]->GetValue()) );
//		m_speed_pid.SetSetPoint( TunableInt::AsFloat(tunableints[8]->GetValue()) + TunableInt::AsFloat(tunableints[13]->GetValue()) );
//
//	}
//	else
//	{
//		gpio_set(PTB22, 1);
//		m_turn_pid.SetKP( TunableInt::AsFloat( tunableints[6]->GetValue()) );
//		m_turn_pid.SetKD( TunableInt::AsFloat( tunableints[7]->GetValue()) );
//		m_speed_pid.SetSetPoint(  TunableInt::AsFloat(tunableints[8]->GetValue()) - TunableInt::AsFloat(tunableints[15]->GetValue())*s_error - TunableInt::AsFloat(tunableints[14]->GetValue()) );
//	}


	float degree = round(
		m_turn_pid.Proportional() +  m_turn_pid.Integral() + m_turn_pid.Derivative()
	);

	//degree = Clamp<int32_t>(-90, degree, 90);

	m_turn_pid.UpdatePreviousError();

	turn_smoothing.UpdateCurrentOutput( -degree * 10 );

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
	m_car.MoveMotor(0,(uint16_t) abs(m_total_speed[0]));

	m_car.MotorDir(1, m_dir[1]);			////Left Motor - False Backward  -  True Forward
	m_car.MoveMotor(1,(uint16_t) abs(m_total_speed[1]));
}

void CameraApp::AutoMode(int mode)
{
	uint32_t pt = libutil::Clock::Time();

	while (true)
	{

		if((int) TunableInt::AsFloat(tunableints[18]->GetValue())==1) {
			eStop();
		}

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(), t)>0){
			t = libutil::Clock::Time();

			if(mode != -1){
				if(libutil::Clock::TimeDiff(t, pt) > 3000){
					mode_chosen = mode;
					m_speed_pid.SetSetPoint( SPEED_SETPOINTS[mode_chosen] );
				}
			}

//		if(!stopped && t%SPEEDINPUTPERIOD==0 && m_speed_pid.GetSetPoint() < 1){
//
//				speed_input_smoothing.UpdateCurrentOutput( TunableInt::AsFloat(tunableints[8]->GetValue()) );
//				/*printf("b_kp: %f\r\n",b_kp[1]);
//				printf("b_kd: %f\r\n",b_kd[1]);
//				printf("b_ki: %f\r\n",b_ki[1]);
//				printf("s_kp: %f\r\n",s_kp[1]);
//				printf("s_kd: %f\r\n",s_kd[1]);
//				printf("s_ki: %f\r\n",s_ki[1]);
//				printf("t_kp: %f\r\n",t_kp[1]);
//				printf("t_kd: %f\r\n",t_kd[1]);
//				printf("SPEED_SETPOINTS: %f\r\n",SPEED_SETPOINTS[1]);*/
				//printf("degree: %.3f\r\n",m_gyro);
//			}

			if(!stopped && m_speed_pid.GetSetPoint() < 1) {
				if(mode_chosen==-1)
					m_speed_pid.SetSetPoint( TunableInt::AsFloat(tunableints[8]->GetValue()) );
			}

			///Speed Control Output every 1ms///
			SpeedControlOutput();

			///Turn Control Output every 1 ms///
			TurnControlOutput();


			if(t%2==0){
				mpu6050_update();
				BalanceControl();
			}

			if(t%100==0) printf("Plot:%g,%g,%g\n", angle[0], m_car.GetRawAngle(), m_gyro);

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
//				if(m_speed_pid.GetSetPoint() > 1)
					TurnControl();
			}

			///Speed PID update every 20ms///
			if(t%SPEEDCONTROLPERIOD==0){
				SpeedControl();
			}
			if(t%5==0) {
				MoveMotor();
			}
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
	int32_t encoder1, encoder2;
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
						/*printf("b_kp: %f\r\n",b_kp[1]);
						printf("b_kd: %f\r\n",b_kd[1]);
						printf("b_ki: %f\r\n",b_ki[1]);
						printf("s_kp: %f\r\n",s_kp[1]);
						printf("s_kd: %f\r\n",s_kd[1]);
						printf("s_ki: %f\r\n",s_ki[1]);
						printf("t_kp: %f\r\n",t_kp[1]);
						printf("t_kd: %f\r\n",t_kd[1]);
						printf("SPEED_SETPOINTS: %f\r\n",SPEED_SETPOINTS[1]);*/
					}

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				// TurnControlOutput();


				if(t%2==0){
					mpu6050_update();
					BalanceControl();
				}

				if(t%2==1){
//
//					BalanceControl();
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
//						TurnControl();
					}

//				}


				///Speed PID update every 20ms///
				if(t%SPEEDCONTROLPERIOD==0){
	//				printf("DT: %d\r\n", libutil::Clock::TimeDiff(libutil::Clock::Time(),dt));
	//				SpeedControl();
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
					/*printf("b_kp: %f\r\n",b_kp[1]);
					printf("b_kd: %f\r\n",b_kd[1]);
					printf("b_ki: %f\r\n",b_ki[1]);
					printf("s_kp: %f\r\n",s_kp[1]);
					printf("s_kd: %f\r\n",s_kd[1]);
					printf("s_ki: %f\r\n",s_ki[1]);
					printf("t_kp: %f\r\n",t_kp[1]);
					printf("t_kd: %f\r\n",t_kd[1]);
					printf("SPEED_SETPOINTS: %f\r\n",SPEED_SETPOINTS[1]);*/
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


void CameraApp::Run()
{
	m_car.GetLcd()->Clear(WHITE);

	char* title = "Mode:";
	char* choices[9] = {
		"Auto",				//1
		"Competition 1",	//2
		"Competition 2",	//3
		"Competition 3",	//4
		"Camera",			//5
		"Accel & Gyro",		//6
		"Encoder",			//7
		"Balance Only",		//8
		"Balance & Speed",	//9

	};

	LcdMenu lcdmenu(&m_car, title, choices, 9);
	lcdmenu.CreateMenu();
	lcdmenu.WaitForSelection();

	DisableIsr(PIT3_VECTORn);

	m_car.GetLcd()->Clear(WHITE);

	switch(lcdmenu.GetSelectedChoice()){
		case 1:
			AutoMode();
			break;
		case 2:
			AutoMode(0);
			break;
		case 3:
			AutoMode(1);
			break;
		case 4:
			AutoMode(2);
			break;
		case 5:
			CameraMode();
			break;
		case 6:
			AccelAndGyroMode();
			break;
		case 7:
			EncoderMode();
			break;
		case 8:
			BalanceOnlyMode();
			break;
		case 9:
			BalanceAndSpeedMode();
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
