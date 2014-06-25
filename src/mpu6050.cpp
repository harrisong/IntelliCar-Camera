#include <stdint.h>
#include "kalman.h"
#include "vars.h"
#include "mpu6050.h"
#include <mini_common.h>
#include <hw_common.h>
#include <MK60_i2c.h>
#include <MK60_gpio.h>
#include <libutil/clock.h>
#include <libsc/com/config/2014_camera.h>


KF m_gyro_kf[3];
KF m_acc_kf;

int16_t SPEEDSETPOINT = 0;
uint16_t c_time_img = 0;

int16_t omega_offset[3] = {0,0,0};
int32_t gyro_cal_sum[3] = {0,0,0};


unsigned char data[14];
int16_t raw_acc[3] = {0,0,0};
int16_t raw_omega[3] = {0,0,0};
float omega[3] = {0,0,0};
float acc[3] = {0,0,0};
float angle[3] = {18,0,0};
int gyro_cal_ok = 0;

#define GYROLSB 1/16.4f

#ifdef LIBSC_USE_MPU6050
void  mpu6050_update(){

//	sw_i2c_read_nbytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data);
	i2c_read_nbytes(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data);

	for(int i = 8; i < 14; i += 2){
		/*if(i >= 0 && i <= 5){
			int j = i / 2;
			raw_acc[j] = data[i + 1] | (data[i] << 8);
			acc[j] = (float)raw_acc[j] * 0.000244140625f;
		}
		else */
		if(i >= 8 && i <= 13){
			raw_omega[(i - 8) / 2] = data[i + 1] | (data[i] << 8);
			raw_omega[(i - 8) / 2] -= omega_offset[(i - 8) / 2];
			omega[(i - 8) / 2] = (float)raw_omega[(i - 8) / 2] * GYROLSB;
		}
	}
	for(int i = 0; i < 3; i++){
		omega[i] = omega[i] < 5.0f && omega[i] > -5.0f ? 0 : omega[i];
	}
	if(gyro_cal_ok){
		kalman_filtering(m_gyro_kf, omega, 3);
	}

	for(int i = 0; i < 3; i++){
//		if(i==0) angle[i] -= omega[i] * 0.002f / 2;
//		else
		angle[i] += omega[i] * 0.002f / 2;
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
//	printf("omega_offset:%d,%d,%d\n", omega_offset[0], omega_offset[1], omega_offset[2]);
	gyro_cal_ok = 1;

}

void mpu6050_init(){
	i2c_init(I2C1, 400000);
	DELAY_MS(1000);
//	printf("init start\n");

	/*sw_i2c_write(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);		//use PLL with Z axis gyro ref
	sw_i2c_write(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00);		//sampling frequence=1000K
	sw_i2c_write(MPU6050_ADDRESS, MPU6050_CONFIG, 0x00);			//bandwith: gyro=256hz, acc=260hz
	sw_i2c_write(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x10);	//gyro range: 00->250, 08->500, 10->1000, 18->2000
	sw_i2c_write(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);	//acc range: 00->2g, 08->4g, 10->8g, 18->16g*/

	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);		//use PLL with Z axis gyro ref
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00);		//sampling frequence=1000K
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_CONFIG, 0x00);			//bandwith: gyro=256hz, acc=260hz
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x10);	//gyro range: 00->250, 08->500, 10->1000, 18->2000
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);	//acc range: 00->2g, 08->4g, 10->8g, 18->16g

	DELAY_MS(1000);
	gyro_cal();
//	printf("init ends\n");

}
#else

void mpu6050_init(){}
void gyro_cal(void){}
void  mpu6050_update(){}

#endif
