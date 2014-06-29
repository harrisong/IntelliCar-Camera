#include <stdint.h>
#include "kalman.h"
#include "vars.h"
#include "mpu6050.h"
#include <mini_common.h>
#include <hw_common.h>
#include <MK60_i2c.h>
#include <MK60_gpio.h>
#include <libutil/clock.h>
#include "MK60_port.h"
#include "MK60_i2c.h"

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

#define i2c_Start(I2Cn)             I2C_C1_REG(I2C1_BASE_PTR) |= (I2C_C1_TX_MASK | I2C_C1_MST_MASK)    //MST 由0变1，产生起始信号，TX = 1 进入发送模式

//停止信号
#define i2c_Stop(I2Cn)              I2C_C1_REG(I2C1_BASE_PTR) &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK)   //MST 由1变0，产生停止信号，TX = 0 进入接收模式

//重复启动
#define i2c_RepeatedStart(I2Cn)     I2C_C1_REG(I2C1_BASE_PTR) |= I2C_C1_RSTA_MASK

//进入接收模式(应答,需要接收多个数据，接收最后一个字节前需要禁用应答i2c_DisableAck)
#define i2c_EnterRxMode(I2Cn)       I2C_C1_REG(I2C1_BASE_PTR) &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK)  //

//进入接收模式(不应答,只接收一个字节)
#define i2c_PutinRxMode(I2Cn)       I2C_C1_REG(I2C1_BASE_PTR) &= ~I2C_C1_TX_MASK;I2C_C1_REG(I2C1_BASE_PTR) |= I2C_C1_TXAK_MASK

//禁用应答(接收最后一个字节)
#define i2c_DisableAck(I2Cn)        I2C_C1_REG(I2C1_BASE_PTR) |= I2C_C1_TXAK_MASK

//等待 I2C_S
#define i2c_Wait(I2Cn)              while(( I2C_S_REG(I2C1_BASE_PTR) & I2C_S_IICIF_MASK)==0) {} \
                                    I2C_S_REG(I2C1_BASE_PTR) |= I2C_S_IICIF_MASK;

//写一个字节
#define i2c_write_byte(I2Cn,data)   (I2C_D_REG(I2C1_BASE_PTR) = (data));i2c_Wait(I2Cn)


volatile uint16_t tick_img = 0;
volatile uint16_t _timeout = 0;

void set_timeout(uint16_t value){

	tick_img = libutil::Clock::Time();
	_timeout = value;
}

uint8_t timeout(void){
	if(libutil::Clock::TimeDiff(libutil::Clock::Time(),tick_img) > 0){
		tick_img = libutil::Clock::Time();
		_timeout--;
	}
	return _timeout > 0 ? 0 : 1;
}

void my_i2c_reset_sda(){
	gpio_init(PTE1, GPO, 1);
	DELAY_US(1);
	gpio_set(PTE1, 0);
	DELAY_US(1);
	i2c_init(I2C1, 400000);
	DELAY_MS(1);;
}

uint8_t my_i2c_Wait(I2Cn) {
	set_timeout(3);
	while(( I2C_S_REG(I2C1_BASE_PTR) & I2C_S_IICIF_MASK)==0) {
		if(timeout()){
			gpio_set(PTA11, 1);
			gpio_init(PTD0, GPO, 0);
			my_i2c_reset_sda();
			return 0;
		}
	}
	I2C_S_REG(I2C1_BASE_PTR) |= I2C_S_IICIF_MASK;
	return 1;
}

uint8_t my_i2c_read_nbytes(I2Cn_e i2cn, uint8_t addr, uint8_t reg, int length, uint8_t * pdata){
	int i = 0;
	uint8_t dump;
	i2c_Start(i2cn);
	i2c_write_byte(i2cn, ( addr << 1 ) | MWSR);
	i2c_write_byte(i2cn, reg);
    i2c_RepeatedStart(i2cn);
	i2c_write_byte(i2cn, ( addr << 1 ) | MRSW);
    i2c_EnterRxMode(i2cn);
    dump = I2C_D_REG(I2C1_BASE_PTR);
	if(!my_i2c_Wait(i2cn)){
		return 0;
	}
	for(i = 0; i < length; i++){
		if(i == length - 1){
			i2c_PutinRxMode(i2cn);
			*(pdata + i) = I2C_D_REG(I2C1_BASE_PTR);
		}
		else{
			*(pdata + i) = I2C_D_REG(I2C1_BASE_PTR);
		}
		if(!my_i2c_Wait(i2cn)){
			return 0;
		}
	}
	i2c_Stop(i2cn);
	return 1;
}

void mpu6050_short_init(){
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);		//use PLL with Z axis gyro ref
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00);		//sampling frequence=1000K
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_CONFIG, 0x00);			//bandwith: gyro=256hz, acc=260hz
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x10);	//gyro range: 00->250, 08->500, 10->1000, 18->2000
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);	//acc range: 00->2g, 08->4g, 10->8g, 18->16g

}

void  mpu6050_update(){

//	sw_i2c_read_nbytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data);
//	i2c_read_nbytes(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data);
	if(!my_i2c_read_nbytes(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, data)){
		mpu6050_short_init();
		gpio_set(PTA11, 0);
	}
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
