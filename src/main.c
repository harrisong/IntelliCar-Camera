/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ�����?����������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      Ұ��K60 ƽ̨������
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */


#include "include.h"
#include "math.h"
#include <syscall.h>
#include <vectors.h>

int MyFwriteHandler(int haha, char *ptr, int len)
{
	uart_putbuff(UART3, (uint8_t*)ptr, len);
	return len;
}

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint8 img[CAMERA_W*CAMERA_H];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��

//��������
void sendimg(uint8 *imgaddr, uint32 imgsize);          //����ͼ����λ��
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();

/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note       Ұ�� DMA �ɼ�����ͷ ʵ��
                ע�⣬����� busƵ����Ϊ100MHz(50MHz busƵ�ʻ�̫�����û����ʱ�ɼ�ͼ��)
 */
int  main(void)
{
	uart_init(UART3, 115200);
	__g_fwrite_handler = MyFwriteHandler;
    //��ʼ������ͷ
    camera_init(imgbuff);

    //�����жϸ�λ����
    SetIsr(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
    SetIsr(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler

    while(1)
    {
        //��ȡͼ��
        camera_get_img();                                   //����ͷ��ȡͼ��

        //��ѹͼ��
        img_extract(img, imgbuff,CAMERA_SIZE);

        //����ͼ����λ��
        sendimg(img, CAMERA_W * CAMERA_H);                  //���͵���λ��
        //DELAY_MS(100);
    }
}

/*!
 *  @brief      ����ͼ��eSmartCameraCar��λ����ʾ
 *  @param      imgaddr         ͼ���ַ
 *  @param      imgsize         ͼ��ռ�ÿռ��С
 *  @since      v5.0
 *  @note       ��ͬ����λ��ͬ���������ʹ�� eSmartCameraCar�����
                ���ʹ��������λ������Ҫ�޸Ĵ��롣
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_����ͷ���ڵ��� ʹ�õ�����

    uart_putbuff(FIRE_PORT, cmd, sizeof(cmd));    //�ȷ�������

    uart_putbuff(FIRE_PORT, imgaddr, imgsize); //�ٷ���ͼ��
}

/*!
 *  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
 *  @param      dst             ͼ���ѹĿ�ĵ�ַ
 *  @param      src             ͼ���ѹԴ��ַ
 *  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��Ұ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
    uint8 tmpsrc;
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

/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //��ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
