/*
 * balance_gyro.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_gyro.h>
#include <FIRE_MK60_conf.h>
#include <float.h>

#include <mini_common.h>
#include <hw_common.h>
#include <MK60_pit.h>
#include <vectors.h>

//#define TIMECONST 0.2f
#define TIMECONST 8.0f
#define DT 20
#define ITERATIONS 200

#define Q_angle	2.7f //0.28  //0.091500963 //gyro vari
#define	Q_bias	0.0000107f      //0.003  //0.004 //gyro shift
#define	R_measure	0.22f //0.000833 //0.000384534 //accel vari

float  angle = 0; // Reset the angle
float  bias = 0; // Reset bias
float 	innoY;
float 	S;
float 	P[2][2];
float   K[2];
float		 rate;



float getAngle(float newAngle, float newRate, float dt) {

       float rate = newRate - bias;
				P[0][0] = 0;
				P[0][1] = 0;
				P[1][0] = 0;
				P[1][1] = 0;
       angle += dt * rate;
       // Update estimation error covariance - Project the error covariance ahead
       P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
       P[0][1] -= dt * P[1][1];
       P[1][0] -= dt * P[1][1];
       P[1][1] += Q_bias * dt;
			 // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
       S = P[0][0] + R_measure;
       // Calculate Kalman gain - Compute the Kalman gain
			 K[0] = P[0][0] / S;
       K[1] = P[1][0] / S;
       // Calculate angle and bias - Update estimate with measurement zk (newAngle)
       innoY = newAngle - angle;
       angle += K[0] * innoY;
       bias += K[1] * innoY;
       // Calculate estimation error covariance - Update the error covariance
       P[0][0] -= K[0] * P[0][0];
       P[0][1] -= K[0] * P[0][1];
       P[1][0] -= K[1] * P[0][0];
       P[1][1] -= K[1] * P[0][1];

			return angle;
    }

float drift_offset=0, drift_offset_error=0;
int acount=0;
__ISR void Pit2Handler()
{

	if(acount<ITERATIONS){

		drift_offset += ( ((float) adc_once(ADC0_SE15, ADC_10bit) * 3.3 / 1023 - 1.795) / 0.00268f )*DT/1000;
		acount++;
	}

	if(acount==ITERATIONS) drift_offset_error = drift_offset * 50/(ITERATIONS);
	PIT_Flag_Clear(PIT2);
}

BalanceGyro::BalanceGyro(ADCn_Ch_e p1, ADCn_Ch_e p3, ADCn_Ch_e p4, int16 sp):
	raw_gyro_port(p1), raw_z_port(p4), raw_x_port(p3),
	raw_setpoint(sp), raw_gyro(0), raw_offset(0), old_raw_offset(0), omega(0),
	raw_gyro_angle(90),
	raw_accel_angle(90),
	Rx(0), Rz(0),
	comp_angle(0),
	c_time(0), p_time(0), p_time_2(0), d_time(0), d_time_2(0),
	Vmax(3.3), Adc16max(65535), Gyrozero(1.793121206),
	Gyroscale(0.00268), ////MAGIC CONSTANT FOR GYRO HERE, REMEMBER
	Accelzero(1.616812667), Accelscale(0.206) {

	adc_init(ADC0_SE14);
	//adc_init(raw_angle_port);
	//adc_init(ADC1_SE4a);
	adc_init(ADC0_SE15);



	SetIsr(PIT2_VECTORn, Pit2Handler);
	pit_init_ms(PIT2, 20);
	EnableIsr(PIT2_VECTORn);


	//printf("Gyrozero Before: %f\n\r", Gyrozero);

	//Gyrozero -= drift_offset_error;
	//printf("Gyrozero After: %f\n\r", Gyrozero);

	int icount=0;
	float total_acce=0;

	while(icount<=50){
		Refresh();

		total_acce+=raw_accel_angle;
		icount++;
		raw_gyro_angle = total_acce/icount;
		init_angle = raw_gyro_angle;
	}

}

void BalanceGyro::Refresh(){
	c_time = libutil::Clock::Time();
	d_time = libutil::Clock::TimeDiff(c_time,p_time);
	d_time_2 = libutil::Clock::TimeDiff(c_time,p_time_2);

	totalsample++;
	//raw_angle = adc_once(raw_angle_port, ADC_16bit);
	//raw_offset = raw_angle - raw_setpoint;
	//omega = raw_offset - old_raw_offset;
	//old_raw_offset = raw_offset;

	//raw_gyro = ((int16) adc_once(raw_gyro_port, ADC_16bit) + raw_gyro)/totalsample;
	////Use Kalman filter now, no need values for complementary filter////

	if(d_time>=DT){

		p_time = c_time;
		raw_gyro = ((float) adc_once(ADC0_SE15, ADC_10bit) * Vmax / 1023 - Gyrozero) / 0.00268f ;
		//Rz =  (((float) adc_once(ADC0_SE15, ADC_10bit) * Vmax/ 1023));// - Accelzero)/ 0.8f;

		raw_gyro_angle += (raw_gyro - drift_offset_error) *DT/1000;
		//printf("%f \t %f\n\r", raw_gyro_angle, drift_offset_error);
		//kalman_angle = getAngle(raw_accel_angle, raw_gyro, 0.02);
		printf("%f, %f, %f, %f\n\r", comp_angle, drift_offset_error, raw_accel_angle, raw_gyro_angle);

	}

		Rx =  (((float) adc_once(ADC0_SE14, ADC_10bit) * Vmax/ 1023) - Accelzero)/ 0.8f;
		if(Rx > 1.0){
			Rx = 1.0;
		}else if(Rx < -1.0){
			Rx = -1.0;
		}
		raw_accel_angle = 90 - (acos(Rx) * 180 / 3.1415f - 90);


		if(d_time_2 >= 8) {
				p_time_2 = c_time;
				comp_angle = 1/TIMECONST * raw_accel_angle + (1- 1/TIMECONST) * raw_gyro_angle;
		}

	//printf("%f, %f\n\r", xangle/3.14*180, raw_gyro);
	//DELAY_MS(100);

}

void BalanceGyro::ChangeSetPoint(uint16 n){
	raw_setpoint = n;
}

float BalanceGyro::get_raw_gyro(){
	return raw_gyro;
}

float BalanceGyro::GetRawAngle(){
	return comp_angle;
}

float BalanceGyro::GetOffset(){
	return raw_offset;
}

float BalanceGyro::GetOmega(){
	return raw_gyro;
}

float BalanceGyro::get_accel(){
	return R;
}

