/*
 * vars.h
 *
 *  Created on: Mar 15, 2014
 *      Author: harrison
 */

#ifndef VARS_H_
#define VARS_H_

#define TIMECONST 100.0f

//balance pid vars//
/*#define balance_kp 7.7f
#define balance_kd 0.0f
#define balance_ki 0.0f*/
#define balance_kp 2500.0f
#define balance_kd 20000.0f
#define balance_ki 0.0f

#define SETPOINT 44
#define DEADZONELOWER 90
#define DEADZONEHIGHER -20

//speed pid vars//
#define speed_kp 0.0f
#define speed_kd 0.0f
#define speed_ki 1.0f
#define SPEEDCONTROLPERIOD 100


#define position_kp 0.7f
#define position_kd 0.0f
#define position_ki 0.0f
#define POSITIONSETPOINT 0


#endif /* VARS_H_ */
