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

#include "common.h"
#include "include.h"
#include <libsc/com/uart_device.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#define MAX_ONCE_TX_NUM     32
#define COM_LEN     2

uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //预多
uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //二值化图像的buf指针，由于开头有 COM_LEN 个字节是留给校验，所以需要加 COM_LEN

uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//非压缩的二值化图像（用于识别）
void    img_extract(uint8_t *dst,uint8_t *src,uint32_t srclen);


//这些变量都是测试用，有需要，请自行修改
//可以是其他文件定义，不一定是这个文件里定义，然后把
uint8_t  var1,var2;
uint16_t var3,var4;
uint32_t var5,var6;

uint8_t img_buff[60][80];
uint8_t i=0,j=0;

//采集图像，并无线发送到调试器上，接收调试器发来的变量值
//PS:如果你想看简单的摄像头驱动代码，看本文底下那些注释
int main(void)
{
	 //gpio_init(PTA29, GPO, 1);
	 //while (1);
	//Site_t site={0,0};						    //显示图像左上角位置
	//Size_t size={CAMERA_W,CAMERA_H};			//显示区域图像大小
  //uart_init (UART3, 115200);
	libsc::UartDevice uart(3, 115200);
	libutil::Clock::Init();
	libutil::InitDefaultFwriteHandler(&uart);
    //LCD_Init(RED);            					//初始化，设置背景为白色
	Ov7725_Init(img_bin_buff);          		//摄像头初始化

  
//gpio_init(PORTB,20,GPO,1);
                  
	while(1)
	{
		ov7725_get_img();
		img_extract((uint8_t *)img_buff,(uint8_t *)img_bin_buff,CAMERA_SIZE);
		uart.SendChar(0);
		uart.SendChar(255);
		uart.SendChar(1);
		uart.SendChar(0);

		uart.SendBuffer((uint8_t*)img_buff, 80 * 60);
	}

	libutil::UninitDefaultFwriteHandler();
}
      
   /* while(1)
    {
        ov7725_get_img();			            //采集图像
        img_extract((uint8_t *)img_buff,(uint8_t *)img_bin_buff,2400);
        uart_putchar (UART3, 0);
        uart_putchar (UART3, 255);
        uart_putchar (UART3,1);
        uart_putchar (UART3,0);
        for(i=0;i<120;i++)
        {
          for(j=0;j<160;j++)
            uart_putchar (UART1, img_buff[i][j]);
        }
    }
		//如下示范就是不会破坏缓存区的操作：（只对缓存区进行读操作）
        //LCD_Img_Binary(site,size,(uint16_t *)img_bin_buff);		//显示图像
    }     


*/
/********************************************************************
注意：
    如果你只想采集图像并显示，只需要使用下面的main函数即可。
    Ov7725_Init 是初始化摄像头函数
    ov7725_get_img 是获取图像，图像的地址是 ：img_bin_buff*/

/*void main(void)
{
	Site_t site={0,0};						    //显示图像左上角位置
	Size_t size={CAMERA_W,CAMERA_H};			//显示区域图像大小

    LCD_Init(RED);            					//初始化，设置背景为白色
     uart_init (UART3, 115200);     
	Ov7725_Init(img_bin_buff);          		//摄像头初始化

    while(1)
    {         uart_putchar (UART3, '0');
        ov7725_get_img();			            //采集图像

		//如下示范就是不会破坏缓存区的操作：（只对缓存区进行读操作）
       // LCD_Img_Binary(site,size,(uint16_t *)img_bin_buff);		//显示图像

    }     
}

/*///-----------------------------------------------------------------------

//由于采集回来的图像是  一个字节8个像素，需要解压成一个字节1个像素来方便图像处理，可以用img_extract 函数

//想解压到： uint8_t img_buf[H][W];  这个 二维数组里。（当然也可以是一维数组： uint8_t img_buf[H*W];  ）
//则需要调用这个函数：
//img_extract((uint8_t *) img_buf,(uint8_t *) img_bin_buff, H*W/8);          //解压为灰度图像，方便发送到上位机显


//img_extract 的代码如下：

//压缩二值化图像解压（空间 换 时间 解压）
void img_extract(uint8_t * dst,uint8_t * src,uint32_t srclen)
{
	uint8_t colour[2]={255,0};	//0 和 1 分别对应的颜色
							//注：野火的摄像头 0 表示 白色，1表示 黑色
	uint8_t tmpsrc;
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

