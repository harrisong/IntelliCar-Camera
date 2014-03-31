#ifndef __SCCB_H
#define __SCCB_H


#define SCL_H()         PTB20_OUT = 1
#define SCL_L()         PTB20_OUT = 0
#define	SCL_DDR_OUT() 	PTB20_DDR = 1
#define	SCL_DDR_IN() 	PTB20_DDR = 0

#define SDA_H()         PTB21_OUT = 1
#define SDA_L()         PTB21_OUT = 0
#define SDA_IN()      	PTB21_IN
#define SDA_DDR_OUT()	PTB21_DDR = 1
#define SDA_DDR_IN()	PTB21_DDR = 0

#define ADDR_OV7725   0x42

//#define SCCB_DELAY()	SCCB_delay(400)
#define SCCB_DELAY()	DELAY_US(100)


void SCCB_GPIO_init(void);
int SCCB_WriteByte( uint16_t WriteAddress , uint8_t SendByte);
int SCCB_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress);

#endif 
