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
#define balance_kp 2600.0f
#define balance_kd 15000.0f
#define balance_ki 0.0f

#define SETPOINT 47
#define DEADZONELOWER 90
#define DEADZONEHIGHER -20

//speed pid vars//
#define speed_kp 26.0f
#define speed_kd 0.0f
#define speed_ki 3.0f
#define SPEEDCONTROLPERIOD 20

#define degree_kp 0.154f
#define degree_kd 0.0155f

#define position_kp 0.7f
#define position_kd 0.0f
#define position_ki 0.0f
#define POSITIONSETPOINT 0


#endif /* VARS_H_ */
