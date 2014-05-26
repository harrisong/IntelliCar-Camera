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
#include <MK60_i2c.h>
#include <libutil/string.h>
#include "mpu6050.h"
#include <libsc/com/lcd.h>


KF m_gyro_kf[3];
KF m_acc_kf;

int16_t SPEEDSETPOINT = 0;

namespace camera
{



uint16_t c_time_img = 0;

int16_t omega_offset[3] = {0,0,0};
int32_t gyro_cal_sum[3] = {0,0,0};

uint8_t data[14];
int16_t raw_acc[3] = {0,0,0};
int16_t raw_omega[3] = {0,0,0};
float omega[3] = {0,0,0};
float acc[3] = {0,0,0};
float angle[3] = {0,90,0};
volatile int gyro_cal_ok = 0;

void  mpu6050_update(){
	i2c_read_nbytes(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data);

	for(int i = 0; i < 14; i += 2){
		if(i >= 0 && i <= 5){
			int j = i / 2;
			raw_acc[j] = data[i + 1] | (data[i] << 8);
			acc[j] = (float)raw_acc[j] * 0.000244140625f;
		}
		else if(i >= 8 && i <= 13){
			raw_omega[(i - 8) / 2] = data[i + 1] | (data[i] << 8);
			raw_omega[(i - 8) / 2] -= omega_offset[(i - 8) / 2];
			omega[(i - 8) / 2] = (float)raw_omega[(i - 8) / 2] * 0.061f;
		}
	}
	for(int i = 0; i < 3; i++){
		omega[i] = omega[i] < 5.0f && omega[i] > -5.0f ? 0 : omega[i];
	}
	if(gyro_cal_ok){
		kalman_filtering(m_gyro_kf, omega, 3);
	}


}

void gyro_cal(void){
	int i = 0;
	for(int j = 0; j < 3; j++){
		gyro_cal_sum[j] = 0;
	}

	uint16_t c_time_img = 0;
	while(i < 512){
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),c_time_img) > 0){
			c_time_img = libutil::Clock::Time();
			if(c_time_img % 2 == 0){
				mpu6050_update();
				if(i >= 256){
					for(int j = 0; j < 3; j++){
						gyro_cal_sum[j] += (uint32_t)raw_omega[j];
					}
				}
				i++;
			}
		}
	}
	for(int j = 0; j < 3; j++){
		omega_offset[j] = (uint16_t)(gyro_cal_sum[j] / 256);
	}
	printf("omega_offset:%d,%d,%d", omega_offset[0], omega_offset[1], omega_offset[2]);
	gyro_cal_ok = 1;

}

void mpu6050_init(){
	printf("init start");
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);		//use PLL with Z axis gyro ref
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00);		//sampling frequence=1000K
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_CONFIG, 0x00);			//bandwith: gyro=256hz, acc=260hz
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x10);	//gyro range: 00->250, 08->500, 10->1000, 18->2000
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);	//acc range: 00->2g, 08->4g, 10->8g, 18->16g

	DELAY_MS(100);
	gyro_cal();
	printf("init ends");

}

CameraApp::CameraApp():
	m_gyro(0), m_balance_speed1(0), m_balance_speed2(0),
	m_control_speed1(0), m_control_speed2(0),
	m_balance_pid(SETPOINT, balance_kp, balance_ki, balance_kd),
	m_speed_pid(SPEEDSETPOINT, speed_kp, speed_ki, speed_kd),
	m_count(0),
	m_total_speed1(0), m_total_speed2(0),
	prev_tempspeed(0), current_tempspeed(0),
	m_lcd(true),
	m_turn1(0), m_turn2(0)
{
	libutil::Clock::Init();
	kalman_filter_init(&m_gyro_kf[0], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[1], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_gyro_kf[2], 0.0012, 0.012, 0, 1);
	kalman_filter_init(&m_acc_kf, 0.0005, 0.05, 0, 1);
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

	static float acc = m_car.GetRawAngle();
	kalman_filtering(&m_acc_kf, &acc, 1);
	m_gyro = 1/TIMECONST * acc + (1- 1/TIMECONST) * angle[1];


	//Speed1 = Speed2 = balance_kp * m_car.GetGyroOffset() + balance_kd * m_car.GetGyroOmega();
	m_balance_speed1 = m_balance_speed2 = m_balance_pid.Calc( (int16_t) m_gyro );


}


void CameraApp::SpeedControl(){
	static int32_t error, total_error;
	m_encoder_2 = -FTM_QUAD_get(FTM2);
	FTM_QUAD_clean(FTM2);

	error = SPEEDSETPOINT - m_encoder_2;
	total_error += error;

	prev_tempspeed = current_tempspeed;
	current_tempspeed = speed_kp * error + speed_ki * total_error;

}

void CameraApp::SpeedControlOutput(){
    static int32_t speed_control_count;

    m_control_speed1 = m_control_speed2 = (current_tempspeed - prev_tempspeed) * (++speed_control_count) / SPEEDCONTROLPERIOD + prev_tempspeed;
    if(speed_control_count==SPEEDCONTROLPERIOD) speed_control_count = 0;
}


void CameraApp::TurnControl(){


}

void CameraApp::TurnControlOutput(){

}

void CameraApp::MoveMotor(){


		m_total_speed1 = m_balance_speed1 + m_control_speed1 + m_turn1;
		m_total_speed2 = m_balance_speed2 + m_control_speed2 + m_turn2;
		m_total_speed2 = m_total_speed2 * 80 / 100;

		if(m_count%5==0) {
			//printf("%f \t %f\n\r", m_gyro, m_car.GetGyro()->GetOmega());
			//printf("Total Speed: %d \t Balance Output Speed: %d\n", m_total_speed1, m_balance_speed1);
		}
		//m_total_speed1 = m_total_speed2 = 0;


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



void CameraApp::Printline(uint8_t y, const char* s){
	uint8_t x=m_lcd.FONT_W;
	while(*s){
		m_lcd.DrawChar(x, y, *s, 0, 0xFFFF);
		x+=m_lcd.FONT_W;
		s++;
	}
}

void CameraApp::PrintPtr(uint8_t y){
	for(int i=0; i<m_lcd.H; i++) m_lcd.DrawChar(0, i, ' ', 0xFFFF, 0xFFFF);
	m_lcd.DrawChar(0, y, '>', 0, 0xFFFF);
}


void CameraApp::Run()
{
	gpio_init(PTD15, GPI, 1);
	gpio_init(PTE9, GPO, 1);
	gpio_init(PTD11, GPI, 1);
	gpio_init(PTD10, GPI, 1);
	gpio_init(PTD14, GPI, 1);

	int maxh = m_lcd.H/m_lcd.FONT_H;
	int maxw = m_lcd.W/m_lcd.FONT_W;

	m_lcd.Clear(0xFFFF);

	i2c_init(I2C1, 400000);
	DELAY_MS(100);
	mpu6050_init();
	gpio_set(PTE9, 0);
	Printline(m_lcd.FONT_H * maxh, "I2C Ready");


	int mode = -1;
	int ptr_pos = 1;
	int maxchoices = 7;



	char* s[maxh];
	s[0] = "Choose Mode:";
	s[1] = "Auto";
	s[2] = "PID";
	s[3] = "Gyro Debug";
	s[4] = "Speed Debug";
	s[5] = "Camera Debug";
	s[6] = "Parade";
	s[7] = "Balance Only";

	PrintPtr(ptr_pos * m_lcd.FONT_H);

	while(mode==-1){
		for(int i=0; i<=maxchoices; i++) Printline(m_lcd.FONT_H * i, s[i]);


		if(gpio_get(PTD10)==0 && ptr_pos + 1 <= maxchoices){
			++ptr_pos;
			PrintPtr(ptr_pos * m_lcd.FONT_H);
			DELAY_MS(100);
		}

		if(gpio_get(PTD11)==0 && ptr_pos - 1 >= 1){
			--ptr_pos;
			PrintPtr(ptr_pos * m_lcd.FONT_H);
			DELAY_MS(100);
		}

		if(gpio_get(PTD14)==0){
			mode = ptr_pos;
		}

	}

	uint16_t t = 0;
	m_lcd.Clear(0XFFFF);
	switch(mode){
	case 1:
		Printline(m_lcd.FONT_H * 0, "AUTO Mode");

		///////////////////////////AUTO///////////////////////////
		while(gpio_get(PTD15)==1);

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

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				TurnControlOutput();


				///Update Gyro every 2ms///
				if(t % 2 == 0)	{
					mpu6050_update();
					for(int i = 0; i < 3; i++){
						angle[i] += omega[i] * 0.002f / 2;
					}
				}

				switch(t%2){
				case 0:
					BalanceControl();
					break;
				case 1:
					TurnControl();
					break;
				default:
					break;
				}

				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				if(t%5==0) MoveMotor();

				m_count++;
			}

		}
		///////////////////////////AUTO///////////////////////////

		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		///////////////////////////Speed///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Speed Mode");
		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				if(t%5==0) MoveMotor();

			}

		}
		///////////////////////////Speed///////////////////////////
		break;
	case 5:
		///////////////////////////Camera///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Camera Mode");

		while (true)
		{

					///System loop - 1ms///
					if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
						t = libutil::Clock::Time();

						static int deg = GetRotationInstruction();


						if(t%150==0) {
							const char* s = libutil::String::Format("Degree: %d",deg).c_str();
							Printline(m_lcd.FONT_H * 1, s);

						}
					}
		}
		///////////////////////////Camera///////////////////////////
		break;
	case 6:
		///////////////////////////Parade///////////////////////////
		Printline(m_lcd.FONT_H * 0, "Parade Mode");
		while(gpio_get(PTD15)==1);

		while (true)
		{

			///System loop - 1ms///
			if(libutil::Clock::TimeDiff(libutil::Clock::Time(),t)>0){
				t = libutil::Clock::Time();

				if(t%1000==0){
					if(SPEEDSETPOINT - 10 >= -100){
						SPEEDSETPOINT-=10;
					}
				}


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

				///Speed Control Output every 1ms///
				SpeedControlOutput();

				///Turn Control Output every 1 ms///
				TurnControlOutput();


				///Update Gyro every 2ms///
				if(t % 2 == 0)	{
					mpu6050_update();
					for(int i = 0; i < 3; i++){
						angle[i] += omega[i] * 0.002f / 2;
					}
				}

				switch(t%2){
				case 0:
					BalanceControl();
					break;
				case 1:
//					static uint16_t cycle = t%10000;
//					if(cycle<=5000){
						m_turn1 = 1000;
						m_turn2 = -m_turn1;
//					}
					break;
				default:
					break;
				}

				///Speed PID update every 100ms///
				if(t%SPEEDCONTROLPERIOD==0) SpeedControl();
				if(t%5==0) MoveMotor();

				m_count++;
			}

		}
		///////////////////////////Parade///////////////////////////
		break;
	case 7:
		Printline(m_lcd.FONT_H * 0, "Balance Only Mode");

				///////////////////////////Balance Only///////////////////////////
				while(gpio_get(PTD15)==1);

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
							for(int i = 0; i < 3; i++){
								angle[i] += omega[i] * 0.002f / 2;
							}
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

						if(t%5==0) MoveMotor();

						m_count++;
					}

				}
				///////////////////////////Balance Only///////////////////////////
		break;
	default:break;
	}







}

}
