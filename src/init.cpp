/*
 * init.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <cstdlib>
#include <init.h>
#include <stdarg.h>

char buf[100];
KF gyro_kf;
float _gyro;
int _count;
///UART Pit///

void prints(UARTn_e uartx, const char * Data, ...){
	va_list arglist;
	char buf[32];
	va_start(arglist, Data);
	vsprintf(buf, Data, arglist);
	va_end(arglist);

	uart_putstr(uartx, buf);
}

__ISR void Pit1Handler(){
	//uart_putstr (UART3 , buf);
	PIT_Flag_Clear(PIT1);
}


///System loop///
__ISR void Pit0Handler()
{
	//init encoders
	static balance_encoder e1(FTM1);
	static balance_encoder e2(FTM2);

	//init gyro with setpoint - gyro(gyro, angle, z, x, setpoint)
	static balance_gyro gyro(GYROADC, ANGLEADC, RZADC, RXADC, 12200);

	//init motors
	int16 speed1, speed2;
	bool dir1, dir2;
	static libsc::Motor motor1(0); //right motor
	static libsc::Motor motor2(1); //left motor

	///Get values from encoder///
	e1.refresh();
	e2.refresh();
	///Get values from gyro///
	gyro.refresh();

	///PD equation
	//speed1 = speed2 = kp * gyro.get_offset() + kd * gyro.get_omega() / 100;
	_gyro = gyro.get_offset();
	kalman_filtering(&gyro_kf, &_gyro, 1);
	speed1 = speed2 = kp * _gyro + kd * gyro.get_omega() / 100;

	dir1 = (speed1<0) ? true : false;
	dir2 = (speed2<0) ? false : true;
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

	//snprintf(buf, 100, "%d\t %d\t %f\t %f\t \r\n", gyro.get_raw_angle(), gyro.get_offset(), gyro.get_raw_gyro(), gyro.get_accel());
	//snprintf(buf, 100, "Plot:%f,%f\n", _gyro, (float)gyro.get_offset());

	//if(_count++ > 20){
	//	_count = 0;
	//	prints(UART3, "Plot:%d,%d\n",  (int)(_gyro * 100), gyro.get_offset() * 100);
	//}
	PIT_Flag_Clear(PIT0);
}
///System loop///


void init(){
	uart_init(UART3, 115200);
	///Starts UART PIT///
	SetIsr(PIT1_VECTORn, Pit1Handler);
	pit_init_ms(PIT1, 50);
	EnableIsr(PIT1_VECTORn);
	///Ends UART PIT///

	///Starts Init System loop///
	kalman_filter_init(&gyro_kf, 0.005, 0.5, 0, 1);
	SetIsr(PIT0_VECTORn, Pit0Handler);
	pit_init_ms(PIT0, 1);
	EnableIsr(PIT0_VECTORn);
	///Ends Init System loop///

}


