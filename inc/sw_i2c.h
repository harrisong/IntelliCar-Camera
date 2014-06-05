#ifndef	__SW_I2C_H
#define	__SW_I2C_H

//#include <LPC17xx.H>

//#include <inttypes.h>
//#include "ticks.h"
//#include "leds.h"
//#include "uart.h"

#include <mini_common.h>
#include <hw_common.h>
#include <inttypes.h>
#include <cstdlib>
#include <MK60_port.h>
#include <MK60_gpio.h>
#include <libutil/clock.h>
#include <MK60_SysTick.h>


#define WRITE 0
#define READ 1

#define _BV(X)	(1 << X)

#define TIMEOUT	100

#define SW_I2C	1

/*
#define SDA	PTE0
#define SCL	PTC0
*/

#define SDA	PTE0
#define SCL	PTE1

#define IN	0
#define OUT	1

#define ACK	0
#define NACK 1

#define DELAYT 100

#define get_ticks libutil::Clock::Time

void sw_i2c_init(void);
void sw_i2c_write(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t sw_i2c_read(uint8_t addr, uint8_t reg);
//void sw_i2c_read_nbytes(unsigned char addr, unsigned char reg, int length, unsigned char * pdata);
void sw_i2c_read_nbytes(uint8_t addr, uint8_t reg, int length, uint8_t * pdata);

#endif
