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
	uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //é¢„åşı
	uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //äºŒåşışı–å›¾şıçşıbufşı‡éşıï¼Œç”±äºåşıå¤´æşı COM_LEN ä¸ªåşışı‚æ˜¯şı™çşışı¡éşıï¼Œæşıä»¥éşıè¦åşı COM_LEN

	uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//şıåşıç¼©çşıäºŒåşışı–å›¾şıïşışı¨äşıè¯†åˆ«ï¼şı
	//è¿™äşışı˜éşışı½æ˜¯æµ‹èşışı¨ïşışı‰éşıè¦ïşıè¯·è‡ªè¡Œä¿®şışı	//şı¯ä»¥şı¯å…¶ä»–æşıä»¶åşıä¹‰ïşıä¸äşıå®šæ˜¯è¿™ä¸ªşı‡ä»¶şıŒåşıä¹‰ïşışı¶åşışışı	/*uint8_t  var1,var2;
	uint16_t var3,var4;
	uint32_t var5,var6;*/

	uint8_t img_buff[60][80];
	uint8_t i=0,j=0;

	uint8_t colour[2]={255,0};	//0 şışı1 şı†åˆ«å¯¹åşışı„éşışışı								//æ³¨ïşışıç«şı„æşışıå¤´ 0 è¡¨ç¤º şı½è‰²ï¼şıè¡¨ç¤º é»‘è‰²
	uint8_t tmpsrc;

	//libsc::UartDevice uart;

};



#endif /* BALANCE_CAMERA_H_ */
