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
#include <libsc/com/lcd.h>
//#include <libutil/string.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#define MAX_ONCE_TX_NUM     32
#define COM_LEN     2
enum Color{WHITE, BLACK};

uint8_t  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //Ԥ��
uint8_t *img_bin_buff = (uint8_t *)(((uint8_t *)&nrf_buff) + COM_LEN);  //��ֵ��ͼ���bufָ�룬���ڿ�ͷ�� COM_LEN ���ֽ�������У�飬������Ҫ�� COM_LEN

uint8_t 	img_buf[CAMERA_W*CAMERA_H];					//��ѹ���Ķ�ֵ��ͼ������ʶ��
void    img_extract(uint8_t *dst,uint8_t *src,uint32_t srclen);


//��Щ�������ǲ����ã�����Ҫ���������޸�
//�����������ļ����壬��һ��������ļ��ﶨ�壬Ȼ���
uint8_t  var1,var2;
uint16_t var3,var4;
uint32_t var5,var6;

bool img_buff[60][80];
uint8_t i=0,j=0;


Color GetPixelColor(uint8_t x, uint8_t y)
{
	return img_buff[x][y] ? WHITE : BLACK;
}

int main(void)
{
	libsc::Lcd lcd(true);
	lcd.Clear(0x7777);
	 //gpio_init(PTA29, GPO, 1);
	 //while (1);
	//Site_t site={0,0};						    //��ʾͼ�����Ͻ�λ��
	//Size_t size={CAMERA_W,CAMERA_H};			//��ʾ����ͼ���С
  //uart_init (UART3, 115200);
	libsc::UartDevice uart(3, 115200);
	libutil::Clock::Init();
	libutil::InitDefaultFwriteHandler(&uart);
    //LCD_Init(RED);            					//��ʼ�������ñ���Ϊ��ɫ
	Ov7725_Init(img_bin_buff);          		//����ͷ��ʼ��

  
//gpio_init(PORTB,20,GPO,1);
                  
	while(1)
	{
		ov7725_get_img();
		img_extract((uint8_t *)img_buff,(uint8_t *)img_bin_buff,CAMERA_SIZE);

		/*uart.SendChar(0);
		uart.SendChar(255);
		uart.SendChar(1);
		uart.SendChar(0);

		uart.SendBuffer((uint8_t*)img_buff, 80 * 60);*/
		for(int i=0; i<60; i++){
			lcd.DrawPixelBuffer(0, i, 80, 1, 0xFFFF, 0x0000, img_buff[i] );
		}

		int BlackCount = 0;
			int BlackSum = 0;
			float LineCenterX = -1;

			float LineCenterXSet[60];

			for(int i=0; i<60; i++)
			{
				for(int j=8; j<88; j++)
				{
					if(GetPixelColor(i, j%80)==BLACK)
					{
						BlackCount++;
						BlackSum+=j;
					}
				}
				LineCenterX = BlackSum / BlackCount;
				LineCenterXSet[i] = LineCenterX;
			}

			float s1=0; float s2=0; float s3=0; float s4=0;
			float slope;

			for(int i=0; i<60; i++)
			{
				s1+=LineCenterXSet[i]*i;
				s2+=LineCenterXSet[i];
				s3+=i;
				s4+=i*i;
			}

			slope = (60*s1 - s2*s3) / (60*s4 - s2*s2);					//from Excel equation for calculating the slope of giving n points

			float sumX=0; float sumY=0;
			float intercept;

			for(int i=0; i<60; i++)
			{
				sumX+=LineCenterXSet[i];
				sumY+=i;
			}

			intercept = (sumY/60) - (slope*sumX/60);


			//lcd.DrawString(0, 100, libutil::String::Format("y = %fx + %f", slope, intercept).c_str(), 0xFFFF);
			//DEBUG_PRINT("\n\nLine equation: y = %fx + %f\n\n", slope, intercept);			//print our y = mx + c;
	}

	libutil::UninitDefaultFwriteHandler();
}



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

