/*
 * balance_camera.h
 *
 *  Created on: Mar 31, 2014
 *      Author: harrison
 */

#ifndef BALANCE_CAMERA_H_
#define BALANCE_CAMERA_H_

#include "common.h"
#include "include.h"
#include <libsc/com/uart_device.h>
#include <libutil/clock.h>
#include <libutil/misc.h>

enum Color{WHITE, BLACK};

class BalanceCamera {
public:
	BalanceCamera();
	~BalanceCamera();
	void img_extract(uint8_t * dst,uint8_t * src,uint32_t srclen);
	void extract_to_buffer();
	uint8_t* GetImageBuff();
	Color GetPixelColor(uint8_t x, uint8_t y);

private:
	#define MAX_ONCE_TX_NUM     32
	#define COM_LEN     2
	uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //预���
	uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //二������图������buf������，由于���头��� COM_LEN 个������是������������，���以���要��� COM_LEN

	uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//������缩���二������图������������识别���
	//这������������是测���������������要���请自行修����	//���以���其他���件���义���不���定是这个���件������义�������������	/*uint8_t  var1,var2;
	uint16_t var3,var4;
	uint32_t var5,var6;*/

	uint8_t img_buff[60][80];
	uint8_t i=0,j=0;

	uint8_t colour[2]={255,0};	//0 ����1 ���别对�������������								//注������火���������头 0 表示 ���色���表示 黑色
	uint8_t tmpsrc;

	//libsc::UartDevice uart;

};



#endif /* BALANCE_CAMERA_H_ */
