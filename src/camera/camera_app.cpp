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
#include "libutil/string.h"


KF m_gyro_kf[3];
KF m_acc_kf;

#define SPEEDCONTROLPERIOD 100

namespace camera
{

#define MPU6050_ADDRESS   0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC     0x07
#define MPU6050_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC     0x09
#define MPU6050_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC     0x0B
#define MPU6050_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USRL     0x14
#define MPU6050_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USRL     0x16
#define MPU6050_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_ZG_OFFS_USRL     0x18
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_FF_THR           0x1D
#define MPU6050_FF_DUR           0x1E
#define MPU6050_MOT_THR          0x1F
#define MPU6050_MOT_DUR          0x20
#define MPU6050_ZRMOT_THR        0x21
#define MPU6050_ZRMOT_DUR        0x22
#define MPU6050_FIFO_EN          0x23
#define MPU6050_I2C_MST_CTRL     0x24
#define MPU6050_I2C_SLV0_ADDR    0x25
#define MPU6050_I2C_SLV0_REG     0x26
#define MPU6050_I2C_SLV0_CTRL    0x27
#define MPU6050_I2C_SLV1_ADDR    0x28
#define MPU6050_I2C_SLV1_REG     0x29
#define MPU6050_I2C_SLV1_CTRL    0x2A
#define MPU6050_I2C_SLV2_ADDR    0x2B
#define MPU6050_I2C_SLV2_REG     0x2C
#define MPU6050_I2C_SLV2_CTRL    0x2D
#define MPU6050_I2C_SLV3_ADDR    0x2E
#define MPU6050_I2C_SLV3_REG     0x2F
#define MPU6050_I2C_SLV3_CTRL    0x30
#define MPU6050_I2C_SLV4_ADDR    0x31
#define MPU6050_I2C_SLV4_REG     0x32
#define MPU6050_I2C_SLV4_DO      0x33
#define MPU6050_I2C_SLV4_CTRL    0x34
#define MPU6050_I2C_SLV4_DI      0x35
#define MPU6050_I2C_MST_STATUS   0x36
#define MPU6050_INT_PIN_CFG      0x37
#define MPU6050_INT_ENABLE       0x38
#define MPU6050_DMP_INT_STATUS   0x39
#define MPU6050_INT_STATUS       0x3A
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_ACCEL_XOUT_L     0x3C
#define MPU6050_ACCEL_YOUT_H     0x3D
#define MPU6050_ACCEL_YOUT_L     0x3E
#define MPU6050_ACCEL_ZOUT_H     0x3F
#define MPU6050_ACCEL_ZOUT_L     0x40
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_TEMP_OUT_L       0x42
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_GYRO_XOUT_L      0x44
#define MPU6050_GYRO_YOUT_H      0x45
#define MPU6050_GYRO_YOUT_L      0x46
#define MPU6050_GYRO_ZOUT_H      0x47
#define MPU6050_GYRO_ZOUT_L      0x48
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60
#define MPU6050_MOT_DETECT_STATUS    0x61
#define MPU6050_I2C_SLV0_DO      0x63
#define MPU6050_I2C_SLV1_DO      0x64
#define MPU6050_I2C_SLV2_DO      0x65
#define MPU6050_I2C_SLV3_DO      0x66
#define MPU6050_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_SIGNAL_PATH_RESET    0x68
#define MPU6050_MOT_DETECT_CTRL      0x69
#define MPU6050_USER_CTRL        0x6A
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_PWR_MGMT_2       0x6C
#define MPU6050_BANK_SEL         0x6D
#define MPU6050_MEM_START_ADDR   0x6E
#define MPU6050_MEM_R_W          0x6F
#define MPU6050_DMP_CFG_1        0x70
#define MPU6050_DMP_CFG_2        0x71
#define MPU6050_FIFO_COUNTH      0x72
#define MPU6050_FIFO_COUNTL      0x73
#define MPU6050_FIFO_R_W         0x74
#define MPU6050_WHO_AM_I         0x75

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
	m_position_pid(POSITIONSETPOINT, position_kp, position_ki, position_kd),
	m_count(0), m_speed_control_count(0),
	m_total_speed1(0), m_total_speed2(0), m_real_total_speed(0),
	current_time(0), prev_time(0), delta_time(0),
	m_real_current_speed(0), m_real_prev_speed(0),
	prev_tempspeed(0), current_tempspeed(0)
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

#define TIMECONST 100.0f

void CameraApp::BalanceControl()
{
	///Get values from gyro///
	m_car.GyroRefresh();
	//if(m_count%100 == 99) printf("%f, %f\n\r", m_car.GetRawAngle(), m_car.GetGyroOmega());
	///PD equation
	static float acc = m_car.GetRawAngle();
	kalman_filtering(&m_acc_kf, &acc, 1);
	m_gyro = 1/TIMECONST * acc + (1- 1/TIMECONST) * angle[1];

	//printf("Plot:%f\n", m_gyro);
	/*if(m_car.GetRawAngle() < DEADZONELOWER || m_car.GetRawAngle() >  DEADZONEHIGHER) {
		m_balance_speed1 = m_balance_speed2 = 0;
	}else{*/
		//Speed1 = Speed2 = balance_kp * m_car.GetGyroOffset() + balance_kd * m_car.GetGyroOmega();
		m_balance_speed1 = m_balance_speed2 = m_balance_pid.Calc( (int16_t) m_gyro );
	//}

	//printf("Angle:%d, Speed:%d\n\r", (int16_t) m_gyro, m_balance_speed1);


	//m_count++;

}


void CameraApp::SpeedControl(){
	current_time = libutil::Clock::Time();
	delta_time = current_time - prev_time;
	static int32_t error, total_error;

	prev_time = current_time;
	//m_real_current_speed = (FTM_QUAD_get(FTM1) + (-FTM_QUAD_get(FTM2)))/2;
	m_real_current_speed = - FTM_QUAD_get(FTM2);
	if(libutil::Clock::Time()%100==0) printf("%d \n", m_real_current_speed);
	//int16_t m_real_delta_speed = m_real_current_speed;
	//FTM_QUAD_clean(FTM1);
	FTM_QUAD_clean(FTM2);
	//m_real_prev_speed = m_real_current_speed;
	error = SPEEDSETPOINT - m_real_current_speed;
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

void CameraApp::MoveMotor(){

		//m_total_speed1 = m_balance_speed1 - m_speed_speed1;
		//m_total_speed2 = m_balance_speed2 - m_control_speed2;

		m_total_speed1 = m_balance_speed1 + m_control_speed1;
		m_total_speed2 = m_balance_speed2 + m_control_speed2;
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






void CameraApp::Run()
{

	gpio_init(PTD11, GPI, 1);
	gpio_init(PTD15, GPI, 1);
	gpio_init(PTE9, GPO, 1);


	i2c_init(I2C1, 400000);
	DELAY_MS(100);
	mpu6050_init();
	gpio_set(PTE9, 0);



	while(gpio_get(PTD15)==1);

	uint16_t p_time=0, c_time=0;

	while (true)
	{
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),c_time_img) > 0){
			c_time_img = libutil::Clock::Time();
			if(c_time_img % 2 == 0){
				mpu6050_update();
				for(int i = 0; i < 3; i++){
					angle[i] += omega[i] * 0.002f / 2;
				}
			}
		}

		if(c_time_img % 250 == 0){
			//printf("angle:%g,%g,%g\n", angle[0], angle[1], angle[2]);
//			printf("omega:%g,%g,%g\n", omega[0], omega[1], omega[2]);
//			printf("acc:%g,%g,%g\n", acc[0], acc[1], acc[2]);
		}

		///System loop - 1ms///
		if(libutil::Clock::TimeDiff(libutil::Clock::Time(),prev_time)>0){
			prev_time = libutil::Clock::Time();

			///Speed PID update every 100ms///
			if(prev_time%SPEEDCONTROLPERIOD==0) SpeedControl();
			SpeedControlOutput();
			BalanceControl();

			///Update Gyro every 2ms///
			if(prev_time % 2 == 0)	mpu6050_update();

//			switch(prev_time%5){
//			///1st ms - Balance///
//			case 0:
//
//				break;
//			case 1:
//				break;
//			///2nd ms - Turn algorithm///
//			case 2:
//				TurnControl();
//				break;
//			case 3:
//			case 4:
//			default:
//				break;
//			}

			m_count++;

			MoveMotor();

		}

	}

}

}
