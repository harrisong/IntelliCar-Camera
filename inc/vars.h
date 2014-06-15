/*
 * vars.h
 *
 *  Created on: Mar 15, 2014
 *      Author: harrison
 */

#include "kalman.h"
#include <stdint.h>

#ifndef VARS_H_
#define VARS_H_

#define TIMECONST 100.0f

//balance pid vars//
/*#define balance_kp 7.7f
#define balance_kd 0.0f
#define balance_ki 0.0f*/
/*
#define balance_kp 3000.0f
#define balance_kd 30000.0f
#define balance_ki 0.0f
*/

#define num_of_modes 3


extern float b_kp[num_of_modes];
extern float b_kd[num_of_modes];
extern float b_ki[num_of_modes];

#define SETPOINT 54
#define DEADZONELOWER 90
#define DEADZONEHIGHER -20

//speed pid vars//
//#define speed_kp 125.0f
//#define speed_kp 95.0f
/*
#define speed_kp 150.0f
#define speed_kd 0.0f
//#define speed_ki 7.5f
#define speed_ki 11.0f
*/

extern int16_t SPEEDSETPOINTS[num_of_modes];

extern float s_kp[num_of_modes];
extern float s_kd[num_of_modes];
extern float s_ki[num_of_modes];

#define SPEEDCONTROLPERIOD 20

/*
#define degree_kp 0.089f
#define degree_kd 0.05f
*/

extern float t_kp[num_of_modes];
extern float t_kd[num_of_modes];

#define TURNCONTROLPERIOD 8

//#define CAM_W 160
//#define CAM_H 120
#define CAM_W 80
#define CAM_H 60
#define WHITE_BYTE 0xFF
#define BLACK_BYTE 0						//online RGB 565 calculator
#define RED_BYTE 0xF800						//http://www.henningkarlsen.com/electronics/calc_rgb565.php

#define INVERTED_BG_COLOR 	0xDE10
#define WHITE 0xFFFF
#define BLACK 0


extern KF m_gyro_kf[3];
extern KF m_acc_kf;

extern int16_t omega_offset[3];
extern int32_t gyro_cal_sum[3];

extern unsigned char data[14];
extern int16_t raw_acc[3];
extern int16_t raw_omega[3];
extern float omega[3];
extern float acc[3];
extern float angle[3];





#endif /* VARS_H_ */
