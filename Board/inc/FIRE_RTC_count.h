/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ�����?����������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       FIRE_RTC_count.h
 * @brief      rtc��ݼ��㺯��ʵ��(��1970-01-01 Ϊ��׼����2099-12-31)
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-08-27
 */


#ifndef _FIRE_RTC_COUNT_H_
#define _FIRE_RTC_COUNT_H_

typedef struct
{
    uint16  year;   //��
    uint8_t   mon;    //��
    uint8_t   day;    //��

    uint8_t   hour;   //ʱ
    uint8_t   min;    //��
    uint8_t   sec;    //��

    uint8_t   invalid;//ʱ����Ч��飨0��ʾʱ����Ч��1��ʾʱ����Ч��
} time_s;


extern uint8_t   Is_LeapYear(uint32 year);               //�ж��Ƿ�Ϊ���꣨���򷵻�1�����򷵻�0��
extern uint32  year2day(uint32 year);                  //��ָ����ݵ�����
extern uint32  time2sec(time_s time);                  //������ʱ�����ʽʱ��תΪ�� 1970 -01-01Ϊ��������
extern void    sec2time(uint32 sec, time_s *time);     //�� 1970-01-01Ϊ�������� תΪ ������ʱ�����ʽʱ��

#endif
