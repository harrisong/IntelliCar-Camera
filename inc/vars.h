/*
 * vars.h
 *
 *  Created on: Mar 15, 2014
 *      Author: harrison
 */

#ifndef VARS_H_
#define VARS_H_

//balance pid vars//
#define balance_kp 6.0f
#define balance_kd 0.27f
//#define balance_kd 0.0f
#define balance_ki 0.0f

#define SETPOINT 30600
#define DEADZONELOWER 20900
#define DEADZONEHIGHER 39000

//speed pid vars//
#define speed_kp 5.0f
#define speed_kd 0.0f
#define speed_ki 0.0f
#define SPEEDSETPOINT 500




#endif /* VARS_H_ */
