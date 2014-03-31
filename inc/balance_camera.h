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

class BalanceCamera {
public:
	BalanceCamera();
	~BalanceCamera();
	void img_extract(uint8_t * dst,uint8_t * src,uint32_t srclen);
	void extract_to_buffer();
	uint8_t GetImageBuff();

private:
	#define MAX_ONCE_TX_NUM     32
	#define COM_LEN     2
	uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //预多
	uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //二值化图像的buf指针，由于开头有 COM_LEN 个字节是留给校验，所以需要加 COM_LEN

	uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//非压缩的二值化图像（用于识别）

	//这些变量都是测试用，有需要，请自行修改
	//可以是其他文件定义，不一定是这个文件里定义，然后把
	/*uint8_t  var1,var2;
	uint16_t var3,var4;
	uint32_t var5,var6;*/

	uint8_t img_buff[60][80];
	uint8_t i=0,j=0;

	uint8_t colour[2]={255,0};	//0 和 1 分别对应的颜色
								//注：野火的摄像头 0 表示 白色，1表示 黑色
	uint8_t tmpsrc;

	//libsc::UartDevice uart;

};



#endif /* BALANCE_CAMERA_H_ */
