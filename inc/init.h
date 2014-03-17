/*
 * init.h
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */

#ifndef INIT_H_
#define INIT_H_

#include <vars.h>
#include <cstdint>
#include <mini_common.h>
#include <hw_common.h>
#include <MK60_FTM.h>
#include <MK60_gpio.h>
#include <MK60_pit.h>
#include <MK60_uart.h>
#include <vectors.h>
#include "balance_encoder.h"
#include "balance_gyro.h"
//#include <libsc/com/k60/ftm_utils.h>
#include <libsc/com/motor.h>
//#include <libsc/com/uart_device.h>
#include "kalman.h"

void init();



#endif /* INIT_H_ */
