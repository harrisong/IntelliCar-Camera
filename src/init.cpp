/*
 * init.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <init.h>
#include <vars.h>

//init encoders
balance_encoder e1(FTM1);
balance_encoder e2(FTM2);

//init gyro with setpoint
//gyro(gyro, angle, accel (z), setpoint)
balance_gyro gyro(ADC0_SE12, ADC0_SE13, ADC0_SE17, 12200);

//init motors
libsc::Motor motor1(0);
libsc::Motor motor2(1);


///System loop///
__ISR void Pit0Handler()
{
	///Get values from encoder///
	e1.refresh();
	e2.refresh();
	///Get values from gyro///
	gyro.refresh();

	///PD equation
	motorspeed1 = kp * gyro.get_offset() + kd * gyro.get_raw_gyro();
	motorspeed2 = kp * gyro.get_offset() + kd * gyro.get_raw_gyro();

	///Set motors
	motor1.SetPower(motorspeed1);
	motor2.SetPower(motorspeed2);

	PIT_Flag_Clear(PIT0);
}
///System loop///


void init(){
	///Starts Init System loop///
	SetIsr(PIT0_VECTORn, Pit0Handler);
	pit_init_ms(PIT0, 500);
	EnableIsr(PIT0_VECTORn);
	///Ends Init System loop///
}


