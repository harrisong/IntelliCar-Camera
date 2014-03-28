#include "common.h"
#include  <MK60_gpio.h>
#include "SCCB.h"
#include <libutil/misc.h>

#define DEV_ADR  ADDR_OV7725 			 /*�豸��ַ����*/

/*******************************************************************************
* Function Name  : SCCB_delay
* Description    : �ӳ�ʱ��
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void SCCB_delay(volatile uint16_t i)
{	
   while(i) 
   { 
     i--; 
   } 
}


/********************************************************************
 * ������SCCB_Configuration
 * ����  ��SCCB�ܽ�����
 * ����  ����
 * ���  ����
 * ע��  ����        
 ********************************************************************/
void SCCB_GPIO_init(void)
{
	gpio_init  (PTB20, GPO, 0);  //��ʼ��SCL
	gpio_init  (PTB11, GPO, 0);  //��ʼ��SDA
}


/********************************************************************
 * ������SCCB_Start
 * ����  ��SCCB��ʼ�ź�
 * ����  ����
 * ���  ����
 * ע��  ���ڲ�����        
 ********************************************************************/
static uint8_t SCCB_Start(void)
{
	SDA_H();
	SCL_H();
	SCCB_DELAY();
	
	SDA_DDR_IN();
	if(!SDA_IN())
	{
		SDA_DDR_OUT();	
		return 0;	/* SDA��Ϊ�͵�ƽ������æ,�˳� */
	}
	SDA_DDR_OUT();
	SDA_L();
	
	SCCB_DELAY();
	
	SDA_DDR_IN();
	if(SDA_IN()) 
	{
		SDA_DDR_OUT();
		return 0;	/* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
	}
	SDA_DDR_OUT();
	SDA_L();
	//SCCB_delay();
	return 1;
}



/********************************************************************
 * ������SCCB_Stop
 * ����  ��SCCBֹͣ�ź�
 * ����  ����
 * ���  ����
 * ע��  ���ڲ�����        
 ********************************************************************/
static void SCCB_Stop(void)
{
	SCL_L();
	//SCCB_DELAY();
	SDA_L();
	SCCB_DELAY();
	SCL_H();
	//SCCB_DELAY();
	SDA_H();
	SCCB_DELAY();
}



/********************************************************************
 * ������SCCB_Ack
 * ����  ��SCCBӦ��ʽ
 * ����  ����
 * ���  ����
 * ע��  ���ڲ�����        
 ********************************************************************/
static void SCCB_Ack(void)
{	
	SCL_L();
	SCCB_DELAY();
	SDA_L();
	SCCB_DELAY();
	SCL_H();
	SCCB_DELAY();
	SCL_L();
	SCCB_DELAY();
}



/********************************************************************
 * ������SCCB_NoAck
 * ����  ��SCCB ��Ӧ��ʽ
 * ����  ����
 * ���  ����
 * ע��  ���ڲ�����        
 ********************************************************************/
static void SCCB_NoAck(void)
{	
	SCL_L();
	SCCB_DELAY();
	SDA_H();
	SCCB_DELAY();
	SCL_H();
	SCCB_DELAY();
	SCL_L();
	SCCB_DELAY();
}




/********************************************************************
 * ������SCCB_WaitAck
 * ����  ��SCCB �ȴ�Ӧ��
 * ����  ����
 * ���  ������Ϊ:=1��ACK,=0��ACK
 * ע��  ���ڲ�����        
 ********************************************************************/
static int SCCB_WaitAck(void) 	
{
	SCL_L();
	SCCB_DELAY();
	SDA_H();	
	SCCB_DELAY();
	
	SCL_H();
	
	SDA_DDR_IN();
	SCCB_DELAY();

	if(SDA_IN())
	{
	      SDA_DDR_OUT();
      	      SCL_L();
      	      return 0;
	}
	SDA_DDR_OUT();
	SCL_L();
	return 1;
}



 /*******************************************************************
 * ������SCCB_SendByte
 * ����  ����ݴӸ�λ����λ
 * ����  ��SendByte: ���͵����
 * ���  ����
 * ע��  ���ڲ�����        
 *********************************************************************/
static void SCCB_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L();
        SCCB_DELAY();
      	if(SendByte & 0x80)
		{
			SDA_H(); 
		}
      	else 
		{
			SDA_L();   
		}
        DEBUG_PRINT("Warning:SCCB4\n");
        SendByte <<= 1;
        SCCB_DELAY();
		SCL_H();
        SCCB_DELAY();
    }
    SCL_L();
}




 /******************************************************************
 * ������SCCB_ReceiveByte
 * ����  ����ݴӸ�λ����λ
 * ����  ����
 * ���  ��SCCB���߷��ص����
 * ע��  ���ڲ�����        
 *******************************************************************/
static int SCCB_ReceiveByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H();	
	SCCB_DELAY();
	SDA_DDR_IN();	
	
    while(i--)
    {
      ReceiveByte <<= 1;      
      SCL_L();
      SCCB_DELAY();
	  SCL_H();
      SCCB_DELAY();	
	  
      if(SDA_IN())
      {
        ReceiveByte |= 0x01;
      }
	  
    }
	SDA_DDR_OUT();
    SCL_L();
    return ReceiveByte;
}





 /*****************************************************************************************
 * ������SCCB_WriteByte
 * ����  ��дһ�ֽ����
 * ����  ��- WriteAddress: ��д���ַ 	- SendByte: ��д�����	- DeviceAddress: ��������
 * ���  ������Ϊ:=1�ɹ�д��,=0ʧ��
 * ע��  ����        
 *****************************************************************************************/           
static int SCCB_WriteByte_one( uint16_t WriteAddress , uint8_t SendByte );


int SCCB_WriteByte( uint16_t WriteAddress , uint8_t SendByte )            //���ǵ���sccb�Ĺܽ�ģ�⣬�Ƚ�����ʧ�ܣ���˶��Լ���
{
    uint8_t i= 0;
    while( 0 == SCCB_WriteByte_one ( WriteAddress, SendByte ) )
    {
        i++;
        if(i == 20)
        {   DEBUG_PRINT("Warning:SCCB1\n");
            return 0 ;
        }
    }
    return 1;
}

int SCCB_WriteByte_one( uint16_t WriteAddress , uint8_t SendByte )
{		
    if(!SCCB_Start())
	{   
	    return 0;
	}DEBUG_PRINT("Warning:SCCB2\n");
    SCCB_SendByte( DEV_ADR );                    /* ������ַ */
    if( !SCCB_WaitAck() )
	{  
		SCCB_Stop(); 
		return 0;
	}DEBUG_PRINT("Warning:SCCB3\n");
    SCCB_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* ���õ���ʼ��ַ */      
    SCCB_WaitAck();	
    SCCB_SendByte(SendByte);
    SCCB_WaitAck();   
    SCCB_Stop(); 
    return 1;
}									 




/******************************************************************************************************************
 * ������SCCB_ReadByte
 * ����  ����ȡһ�����
 * ����  ��- pBuffer: ��Ŷ������ 	- length: ���������	- ReadAddress: �������ַ		 - DeviceAddress: ��������
 * ���  ������Ϊ:=1�ɹ�����,=0ʧ��
 * ע��  ����        
 **********************************************************************************************************************/           
static int SCCB_ReadByte_one(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress);

int SCCB_ReadByte(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress)
{
    uint8_t i= 0;
    while( 0 == SCCB_ReadByte_one(pBuffer, length, ReadAddress) )
    {
        i++;
        if(i == 20)
        {
            return 0 ;
        }
    }
    return 1;
}

int SCCB_ReadByte_one(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress)
{
    if(!SCCB_Start())
	{
	    return 0;
	}
    SCCB_SendByte( DEV_ADR );         /* ������ַ */
    if( !SCCB_WaitAck() )
	{
		SCCB_Stop(); 
		return 0;
	}
    SCCB_SendByte( ReadAddress );           /* ���õ���ʼ��ַ */      
    SCCB_WaitAck();	
    SCCB_Stop(); 
	
    if(!SCCB_Start())
	{
		return 0;
	}
    SCCB_SendByte( DEV_ADR + 1 );               /* ������ַ */ 
    if(!SCCB_WaitAck())
	{
		SCCB_Stop(); 
		return 0;
	}
    while(length)
    {
      *pBuffer = SCCB_ReceiveByte();
      if(length == 1)
	  {
		  SCCB_NoAck();
	  }
      else
	  {
		SCCB_Ack(); 
	  }
      pBuffer++;
      length--;
    }
    SCCB_Stop();
    return 1;
}




