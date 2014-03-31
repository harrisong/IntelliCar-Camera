/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：main.c
 * 描述         ：SD卡带文件系统实验
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#include "balance_camera.h"

//采集图像，并无线发送到调试器上，接收调试器发来的变量值
//PS:如果你想看简单的摄像头驱动代码，看本文底下那些注释
BalanceCamera::BalanceCamera()
{
	libutil::Clock::Init();
	//libutil::InitDefaultFwriteHandler(&uart);
	Ov7725_Init(img_bin_buff);          		//摄像头初始化

}

BalanceCamera::~BalanceCamera(){
	libutil::UninitDefaultFwriteHandler();
}

void BalanceCamera::extract_to_buffer(){
		ov7725_get_img();
		img_extract((uint8_t *)img_buff,(uint8_t *)img_bin_buff,CAMERA_SIZE);
		/*uart.SendChar(0);
		uart.SendChar(255);
		uart.SendChar(1);
		uart.SendChar(0);

		uart.SendBuffer((uint8_t*)img_buff, 80 * 60);*/
}
      
//img_extract 的代码如下：

//压缩二值化图像解压（空间 换 时间 解压）
void BalanceCamera::img_extract(uint8_t * dst,uint8_t * src,uint32_t srclen)
{

	while(srclen --)
	{
		tmpsrc = *src++;
		*dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}
}

