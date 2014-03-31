#ifndef __OV7725_H
#define __OV7725_H 



#define	CAMERA_DMA_CH 	    DMA_CH0		    //��������ͷ��DMA�ɼ�ͨ��
#define CAMERA_W            80             	//��������ͷͼ����
#define CAMERA_H            60				//��������ͷͼ��߶�
#define CAMERA_INTERLACE    1              	//����ͷ����ɼ����� n - 1,����1��ʾ�����вɼ���2��ʾ���вɼ�

#define CAMERA_DMA_NUM      (CAMERA_W /8 )    //DMA�ɼ�����
#define CAMERA_SIZE         (CAMERA_W * CAMERA_H /8)        //ͼ��ռ�ÿռ��С


#define CAMERA_DMA_LINE     (CAMERA_H/CAMERA_INTERLACE)     //ʵ�ʲɼ�����


extern   uint8_t *	    IMG_BUFF;       //ͼ�񻺳���ָ��         


//����ͼ��ɼ�״̬
typedef enum 
{
    IMG_NOTINIT=0,
	IMG_FINISH,			//ͼ��ɼ����
	IMG_FAIL,				//ͼ��ɼ�ʧ��(�ɼ���������)
	IMG_GATHER,				//ͼ��ɼ���
	IMG_START,				//��ʼ�ɼ�ͼ��
	IMG_STOP,				//��ֹͼ��ɼ�
	
}IMG_STATE;

typedef struct
{
	uint8_t Address;			       /*�Ĵ�����ַ*/
	uint8_t Value;		           /*�Ĵ���ֵ*/
}Register_Info;

extern 	uint8_t Ov7725_vsync;


extern	uint8_t Ov7725_Init(uint8_t *imgaddr);
//extern	void    Ov7725_exti_Init(uint8_t *imgbuff);
//extern	void    ov7727_get_img(uint8_t *imgbuff);
extern	void    Ov7725_exti_Init();
extern	void    ov7725_get_img();

extern	int  	OV7725_ReadReg(uint8_t LCD_Reg,uint16_t LCD_RegValue);
extern	int  	OV7725_WriteReg(uint8_t LCD_Reg,uint16_t LCD_RegValue);

//#define	ARRAY_INDEX(array)		((uint16_t)(sizeof(array)/sizeof(array[0])))
//#define	OV7725_INIT(regcfg)		Ov7725_Init(((Register_Info *)(regcfg)),ARRAY_INDEX(regcfg))


#endif























