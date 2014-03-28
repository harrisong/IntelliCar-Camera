 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,Ұ��Ƽ�
  *     All rights reserved. 
  *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
  *
  *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ�����?����������ҵ��;��
  *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
  *
  * @file       FIRE_RTC_count.c
  * @brief      rtc��ݼ��㺯��ʵ��(��1970-01-01 Ϊ��׼����2099-12-31)
  * @author     Ұ��Ƽ�
  * @version    v5.0
  * @date       2013-08-27
  */


/* 
 * ��ͷ�ļ�
 */
#include "common.h"
#include "FIRE_RTC_count.h"



const uint8_t		mon_t[12]	={ 31,28,31,30,31,30,31,31,30,31,30,31};                //ƽ����·�
const uint8_t		mon2_t[12]	={ 31,29,31,30,31,30,31,31,30,31,30,31};

const uint16	momoff_t[]	={ 0,31,59,90,120,151,181,212,243,273,304,334,365 };    //ƽ���ÿ��1�յ��ۼ�����
const uint16	momoff2_t[]	={ 0,31,60,91,121,152,182,213,244,274,305,335,366 };    //�����ÿ��1�յ��ۼ�����


/*! 
 *  @brief      ����Ƿ�Ϊ����
 *  @param      year    ���
 *  @return     �Ƿ�Ϊ����(0��ʾƽ�꣬1��ʾ����)
 *  @since      v5.0
 *  Sample usage:       if( Is_LeapYear(2013) )         //��� 2013 �Ƿ�Ϊ����
                        {
                            printf("\n����!");          //
                        }else
                        {
                            printf("\n�������꣡");
                        }
 */
uint8_t Is_LeapYear(uint32 year)
{
    if(year%4 == 0)
    {
        if((year%100 != 0) || (year%400 == 0))        
        {
            //����
            return 1;
        }
    }

    //��������
    return 0;
}

/*! 
 *  @brief      ����ݵ�����
 *  @param      year    ���
 *  @return     ָ����ݵ�����
 *  @since      v5.0
 *  Sample usage:       uint32 days = year2day(2013);
                        printf("2013�깲%d��!",days);
 */
uint32 year2day(uint32 year)
{
    return (365 + Is_LeapYear(year));
}

/*! 
 *  @brief      ��1970-01-01 0:0:0 �� ָ��ʱ���������ʱ�����ʽתΪ������
 *  @param      time    ʱ��ṹ��
 *  @return     ��1970-01-01 0:0:0 �� ָ��ʱ���������
 *  @since      v5.0
 *  Sample usage:       time_s time = {2013,9,1,0,0,0};  //ʱ��Ϊ 2013-09-01 0:0:0 
                        printf("��%d��!",time2sec(time));
 */
uint32 time2sec(time_s time)
{
    //�ο� linux�ں˴��� include/linux/time.h ��ĺ��� mktime()
    //��ݸ�˹�㷨����year/mon/day/hour/min/sec����1980��12��31 23��59��59��
    //��ʽ��ʾ��ʱ��ת��Ϊ�����1970��01��01 00��00��00
    //���UNIXʱ���׼�������������
    
    //ע��һ�㣬����Ķ��·���29�죬��������Ϊ28��
    //����һ�򣬰��겻���İ�������
    
    //���ʱ���Ƿ���Ч
    ASSERT(time.year >= 1970);     //���ԣ��� 1970 Ϊ��׼�������СΪ 1970
    ASSERT(time.year < 2100);      //���ԣ�32λ����������ɱ���136���ʱ�䣬���Ǵ˴��޶�������Ϊ2099��ʵ�����ǿɼ�ʱ�� 2105��12��31��
    ASSERT(time.mon  <= 12);       //���ԣ��·����Ϊ12��
    ASSERT(     ((Is_LeapYear(time.year) == 0) &&                    (time.day <= mon_t[time.mon-1])) //ƽ��
           ||   ((Is_LeapYear(time.year) == 1) && (time.mon != 2) && (time.day <= mon_t[time.mon-1]))               //����ķ�2��
           ||   ((Is_LeapYear(time.year) == 1) && (time.mon == 2) && (time.day <= 29))
           );

               
               //ASSERT((time.mon != 2) || ((time.mon == 2) && (time.day  <= 29)));   //���·������ 29�졣
    //ASSERT((time.mon == 2) && (time.year%4 ��= 0)); //��ݷ�4�ı���������ͨ��� 
        
        
    if (0 >= (int8) (time.mon -= 2)) /* 1..12 -> 11,12,1..10 */ 
    {
        time.mon += 12; /* Puts Feb last since it has leap day */ 
        time.year -= 1; 
    }
    
    return ((( 
            (uint32) (time.year/4 - time.year/100 + time.year/400 + 367*(time.mon)/12 + time.day) + time.year*365 - 719499 /* ���е������� */ 
            )*24 + time.hour /* ���е���Сʱ�� */ 
            )*60 + time.min  /* ���е��ܷ����� */ 
            )*60 + time.sec; /* ���յ�������   */ 
}

/*! 
 *  @brief      ��1970-01-01 Ϊ��׼�������� תΪ������ʱ�����ʽ
 *  @param      sec     ������
 *  @param      time    ʱ��ṹ��
 *  @since      v5.0
 *  Sample usage:       time_s time;  
                        sec2time(1000,&time);   //��1000���Ӧ��ʱ��
                        printf("\nʱ���ǣ�%d-%02d-%02d %02d:%02d:%02d",
                                                time.year,time.mon,time.day,
                                                time.hour,time.min,time.sec);
 */
void sec2time(uint32 sec, time_s * time) 
{
#define DAYSEC  (60*60*24)      //����ÿ���������
    //uint32 i;
    uint32  hms                //һ��ʣ�µ�����
            ,day                //�����е�����
			,tmp;

	uint16 const* dayoff ;
	uint8_t  const* monoff;

    if(sec > 0xf48656ff)    //ʱ�䳬�� 2099-12-31 23:59:59�󣬻�� 1970-1-1 0:0:0 ��ʼ��ʱ
    {
        sec -= 0xf48656ff;
    }
    
    day = sec / DAYSEC; 
    hms = sec % DAYSEC; 
    
    //����ʱ����
    time->hour = (uint8_t) (hms / (60*60));         //ʱ
    hms = hms % (60*60);
    time->min = (uint8_t) (hms / 60);  //��
    time->sec = (uint8_t) (hms % 60);   //��
    
    /*�����ǰ��ݣ���ʼ�ļ������Ϊ1970��*/
	time->year  =  (uint16)(day/366 );			//������� ���(��û�� 1970)

	day = day 
            -  365*(time->year)    //����
            - ((time->year +1)/4)  //4��һ����
            //+ (time->year/(2101-1970) )      //2100�������꣬�������� 2099�����˴�ע��
            ;
    
	time->year += 1970;
	tmp = 365 + Is_LeapYear(time->year);        //ָ����ݵ� ����
	if(day >= tmp)
	{
		time->year++;
		day -=  tmp;
	}

	time->mon = (uint8_t)(day/31 +1);					//�������·ݣ�������Ҫ��1

	tmp = Is_LeapYear(time->year);
	dayoff  = tmp ? momoff2_t : momoff_t;
	monoff  = tmp ? mon2_t : mon_t;

	time->day = (uint8_t)(day +1 - dayoff[time->mon -1]);

	if(time->day > monoff[time->mon-1 ] )
	{
		time->mon++;
		time->day = (uint8_t)(day +1 - dayoff[time->mon -1]);
	}
}
