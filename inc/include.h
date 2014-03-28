#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */

#include  <MK60_gpio.h>
#include  <MK60_uart.h>      //����
#include  <MK60_lptmr.h>     //�͹��Ķ�ʱ��(��ʱ)
#include  <MK60_pit.h>
//#include  "LED.h"
#include  <MK60_dma.h>

//#include "BL144002.h"
//#include "lcd.h"
#include "ov7725.h"
#include  <MK60_spi.h>
//#include "NRF24L0.h"
//#include "NRF24L0_MSG.h"
#include "OV7725.h"
//#include "key.h"



extern volatile IMG_STATE	img_flag ;		//ͼ��״̬


#endif  //__INCLUDE_H__
