/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ������
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
#include <vectors.h>
#include <libutil/misc.h>

volatile uint16_t Vnum=0;

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  ������ƣ�PORTA_IRQHandler
*  ����˵����PORTA�˿��жϷ�����
*  ����˵������
*  ����أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
__ISR void PORTA_IRQHandler()
{
    uint8_t  n = 0;    //��ź�
	//Site_t site={10,90};
 //   volatile uint16_t i;
    /*==============================================================================
	ע�⣺
		���жϷ������ж�ǰ�棬�����ȼ��������жϡ�
		���ж��û������ȫ����return��ȷ�����������жϵ��û�����
	
	==============================================================================*/
    n = 29;											//���ж�
    if(PORTA_ISFR & (1 << n))           			//PTA29�����ж�
    {
        DEBUG_PRINT("warning:13"); 
        /*  ����Ϊ�û�����  */
        
        //���ж���Ҫ�ж��ǳ������ǳ���ʼ
        if(img_flag == IMG_START)					//��Ҫ��ʼ�ɼ�ͼ��
        {     DEBUG_PRINT("warning:14");
            img_flag = IMG_GATHER;					//���ͼ��ɼ���
            DisableIsr(PORTA_VECTORn);
             DEBUG_PRINT("warning:15");
            DMA_EN(CAMERA_DMA_CH);            		//ʹ��ͨ��CHn Ӳ������
            
            DMA_DADDR(CAMERA_DMA_CH) = (uint32_t)IMG_BUFF;    //�ָ���ַ
               DEBUG_PRINT("warning:16\n");
        }
        //#ifdef DEBUG
      
        else if(img_flag == IMG_GATHER)				//ͼ��ɼ��н��볡�жϣ���ͼ��ɼ����
        {    DEBUG_PRINT("warning:17");
            while(1);                               //DMA�ɼ��쳣
        }
       // #endif
        else										//ͼ��ɼ�����
        {
        	DisableIsr(PORTA_VECTORn); 						//�ر�PTA���ж�
            //DMA_IRQ_DIS(CAMERA_DMA_CH);	                //�ر�ͨ��CHn �ж����� 
            //LCD_Str(site,"ERROR",BLUE,RED);
            img_flag = IMG_FAIL;					//���ͼ��ɼ�ʧ��
             DEBUG_PRINT("warning:18");
        }
         DEBUG_PRINT("warning:100\n");
        PORTA_ISFR  = ~0;        			        //���ж��ȫ����Ҫ���жϱ�־λ
        return;										//���жϴ����ˣ��Ͳ���Ҫ�������ж�
    
    /*  ����Ϊ�û�����  */
    }

    
    PORTA_ISFR  = ~0;        			//д1���жϱ�־λ
}

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  ������ƣ�PORTE_IRQHandler
*  ����˵����PORTE�˿��жϷ�����
*  ����˵������
*  ����أ���
*  �޸�ʱ�䣺2012-9-17    �Ѳ���
*  ��    ע����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
__ISR void PORTE_IRQHandler()
{
    uint8_t  n;    //��ź�

    n = 27;
    if(PORTE_ISFR & (1 << n))           //PTA26�����ж�
    {
        PORTE_ISFR  |= (1 << n);        //д1���жϱ�־λ
        DisableIsr(PIT1_VECTORn);          //�ر�PIT1�жϣ�����Ӱ������ģ���շ�

        //NRF_Handler();                  //����ģ���жϴ��?��
        EnableIsr(PIT1_VECTORn);          //�ر�PIT1�жϣ�����Ӱ������ģ���շ�
    }
}

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  ������ƣ�PIT0_IRQHandler
*  ����˵����PIT0 ��ʱ�жϷ�����
*  ����˵������
*  ����أ���
*  �޸�ʱ�䣺2012-2-18    �Ѳ���
*  ��    ע��
*************************************************************************/
__ISR void PIT0_IRQHandler(void)
{
	//Site_t site={10,70};
	//disable_irq(87); 							//�ر�PTA���ж�
	//LCD_Num_C(site,Vnum,BLUE,RED);		        //��ʾʵ�ʲɼ���10���֡��
	Vnum=0;
	//enable_irq(87); 							//����PTA���ж�
    PIT_Flag_Clear(PIT0);       				//���жϱ�־λ
}

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  ������ƣ�PIT1_IRQHandler
*  ����˵����PIT1 ��ʱ�жϷ�����
*  ����˵������
*  ����أ���
*  �޸�ʱ�䣺2012-11-18    �Ѳ���
*  ��    ע�������������
*************************************************************************/
__ISR void PIT1_IRQHandler(void)
{
	printf("hello\n");
    //key_IRQHandler();
    PIT_Flag_Clear(PIT1);       				//���жϱ�־λ
}



__ISR void DMA0_IRQHandler()
{
    volatile uint8_t i;
    //img_flag = IMG_FINISH ;
    //disable_irq(87);                      //���жϿ�ʼ�ɼ�ͼƬʱ���͹ر����жϣ�����Ͳ���Ҫ�ٹر�
    DMA_DIS(CAMERA_DMA_CH);            	//�ر�ͨ��CHn Ӳ������
    img_flag = IMG_FINISH ;
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //���ͨ�������жϱ�־λ

  Vnum++;
   i++;
    i++;
    i++;
   i++;                        //��ʱ�����ⳬƵʱ��DMA���ȶ����Ϸ�
}

