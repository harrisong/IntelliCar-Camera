/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��main.c
 * ����         ��SD�����ļ�ϵͳʵ��
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * �����      ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��Ұ��Ƕ��ʽ����������
 * �Ա���       ��http://firestm32.taobao.com
 * ����֧����̳ ��http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#include "balance_camera.h"

//�ɼ�ͼ�񣬲����߷��͵����������������յ����������ı���ֵ
//PS:������뿴�򵥵�����ͷ����룬�����ĵ�����Щע��
BalanceCamera::BalanceCamera()
{
	libutil::Clock::Init();
	//libutil::InitDefaultFwriteHandler(&uart);
	Ov7725_Init(img_bin_buff);          		//����ͷ��ʼ��

}

BalanceCamera::~BalanceCamera(){
	libutil::UninitDefaultFwriteHandler();
}

void BalanceCamera::extract_to_buffer(){
		ov7725_get_img();
		img_extract((uint8_t *)img_buff,(uint8_t *)img_bin_buff,CAMERA_SIZE);
}

uint8_t* BalanceCamera::GetImageBuff(){
	return (uint8_t*)img_buff;
}

Color BalanceCamera::GetPixelColor(uint8_t x, uint8_t y)
{
	return img_buff[x][y] ? BLACK : WHITE;
}
//img_extract �Ĵ�����������

//ѹ����ֵ��ͼ���ѹ���ռ����� ʱ�� ��ѹ��
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

