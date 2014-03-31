/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��main.c
 * ����         ��SD�����ļ�ϵͳʵ��
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��Ұ��Ƕ��ʽ����������
 * �Ա���       ��http://firestm32.taobao.com
 * ����֧����̳ ��http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#include "common.h"
#include "include.h"
#include <libsc/com/uart_device.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#define MAX_ONCE_TX_NUM     32
#define COM_LEN     2

uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //Ԥ��
uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //��ֵ��ͼ���bufָ�룬���ڿ�ͷ�� COM_LEN ���ֽ�������У�飬������Ҫ�� COM_LEN

uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//��ѹ���Ķ�ֵ��ͼ������ʶ��
void    img_extract(uint8_t *dst,uint8_t *src,uint32_t srclen);


//��Щ�������ǲ����ã�����Ҫ���������޸�
//�����������ļ����壬��һ��������ļ��ﶨ�壬Ȼ���
uint8_t  var1,var2;
uint16_t var3,var4;
uint32_t var5,var6;

uint8_t img_buff[60][80];
uint8_t i=0,j=0;

//�ɼ�ͼ�񣬲����߷��͵��������ϣ����յ����������ı���ֵ
//PS:������뿴�򵥵�����ͷ����룬�����ĵ�����Щע��
int main(void)
{
	 //gpio_init(PTA29, GPO, 1);
	 //while (1);
	//Site_t site={0,0};						    //��ʾͼ�����Ͻ�λ��
	//Size_t size={CAMERA_W,CAMERA_H};			//��ʾ����ͼ���С
  //uart_init (UART3, 115200);
	libsc::UartDevice uart(3, 115200);
	libutil::Clock::Init();
	libutil::InitDefaultFwriteHandler(&uart);
    //LCD_Init(RED);            					//��ʼ�������ñ���Ϊ��ɫ
	DEBUG_PRINT("Warning:FUCKFUCKFUCK\n");
	Ov7725_Init(img_bin_buff);          		//����ͷ��ʼ��
	DEBUG_PRINT("Warning:OKAYNOW\n");
  
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
        ov7725_get_img();			            //�ɼ�ͼ��
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
		//����ʾ�����ǲ����ƻ�������Ĳ�������ֻ�Ի�������ж�������
        //LCD_Img_Binary(site,size,(uint16_t *)img_bin_buff);		//��ʾͼ��
    }     


*/
/********************************************************************
ע�⣺
    �����ֻ��ɼ�ͼ����ʾ��ֻ��Ҫʹ�������main����ɡ�
    Ov7725_Init �ǳ�ʼ������ͷ����
    ov7725_get_img �ǻ�ȡͼ��ͼ��ĵ�ַ�� ��img_bin_buff*/

/*void main(void)
{
	Site_t site={0,0};						    //��ʾͼ�����Ͻ�λ��
	Size_t size={CAMERA_W,CAMERA_H};			//��ʾ����ͼ���С

    LCD_Init(RED);            					//��ʼ�������ñ���Ϊ��ɫ
     uart_init (UART3, 115200);     
	Ov7725_Init(img_bin_buff);          		//����ͷ��ʼ��

    while(1)
    {         uart_putchar (UART3, '0');
        ov7725_get_img();			            //�ɼ�ͼ��

		//����ʾ�����ǲ����ƻ�������Ĳ�������ֻ�Ի�������ж�������
       // LCD_Img_Binary(site,size,(uint16_t *)img_bin_buff);		//��ʾͼ��

    }     
}

/*///-----------------------------------------------------------------------

//���ڲɼ�������ͼ����  һ���ֽ�8�����أ���Ҫ��ѹ��һ���ֽ�1������������ͼ���?������img_extract ����

//���ѹ���� uint8_t img_buf[H][W];  ��� ��ά���������ȻҲ������һά���飺 uint8_t img_buf[H*W];  ��
//����Ҫ�����������
//img_extract((uint8_t *) img_buf,(uint8_t *) img_bin_buff, H*W/8);          //��ѹΪ�Ҷ�ͼ�񣬷��㷢�͵���λ����


//img_extract �Ĵ������£�

//ѹ����ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
void img_extract(uint8_t * dst,uint8_t * src,uint32_t srclen)
{
	uint8_t colour[2]={255,0};	//0 �� 1 �ֱ��Ӧ����ɫ
							//ע��Ұ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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

