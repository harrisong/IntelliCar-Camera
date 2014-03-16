/*
 * init.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <cstdlib>
#include <init.h>

char buf[100];

///UART Pit///
__ISR void Pit1Handler(){
	uart_putstr (UART3 , buf);
	PIT_Flag_Clear(PIT1);
}


///System loop///
__ISR void Pit0Handler()
{
	//init encoders
	static balance_encoder e1(FTM1);
	static balance_encoder e2(FTM2);

	//init gyro with setpoint - gyro(gyro, angle, z, x, setpoint)
	static balance_gyro gyro(GYROADC, ANGLEADC, RZADC, RXADC, 12150);

	//init motors
	int16 speed1, speed2;
		bool dir1, dir2;
	static libsc::Motor motor1(0);
	static libsc::Motor motor2(1);

	///Get values from encoder///
	e1.refresh();
	e2.refresh();
	///Get values from gyro///
	gyro.refresh();

	///PD equation
	speed1 = speed2 = kp * gyro.get_offset() / 100 + kd * gyro.get_omega();

	dir1 = (speed1<0) ? false : true;
	dir2 = (speed2<0) ? true : false;
	motor1.SetClockwise(dir1);
	motor2.SetClockwise(dir2);

	if(gyro.get_offset() < -6600 || gyro.get_offset() >  6500) {
		speed1 = 0;
		speed2 = 0;
	}

	speed2*=0.9;
	///Set motors
	motor1.SetPower(abs(speed1));
	motor2.SetPower(abs(speed2));

	snprintf(buf, 100, "%d\t %f\t %f\t \r\n", gyro.get_offset(), gyro.get_raw_gyro(), gyro.get_accel());


	PIT_Flag_Clear(PIT0);
}
///System loop///


void init(){
	uart_init(UART3, 115200);
	///Starts UART PIT///
	SetIsr(PIT1_VECTORn, Pit1Handler);
	pit_init_ms(PIT1, 1000);
	EnableIsr(PIT1_VECTORn);
	///Ends UART PIT///

	///Starts Init System loop///
	SetIsr(PIT0_VECTORn, Pit0Handler);
	pit_init_ms(PIT0, 1);
	EnableIsr(PIT0_VECTORn);
	///Ends Init System loop///

}


