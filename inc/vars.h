/*
 * vars.h
 *
 *  Created on: Mar 15, 2014
 *      Author: harrison
 */

#ifndef VARS_H_
#define VARS_H_

//balance pid vars//
/*#define balance_kp 7.7f
#define balance_kd 0.0f
#define balance_ki 0.0f*/
#define balance_kp 15.0f
#define balance_kd 1.0f
#define balance_ki 0.0f

#define SETPOINT 25360
#define DEADZONELOWER 20000
#define DEADZONEHIGHER 35700

//speed pid vars//
#define speed_kp 20.0f
#define speed_kd 0.0f
#define speed_ki 2.0f
#define SPEEDSETPOINT 250


#define position_kp 0.7f
#define position_kd 0.0f
#define position_ki 0.0f
#define POSITIONSETPOINT 0


#endif /* VARS_H_ */
