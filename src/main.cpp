#include "jsmn/jsmn.h"
//#include <string>
#include <cstdint>
#include <mini_common.h>
#include <hw_common.h>
#include <MK60_SysTick.h>
#include <MK60_port.h>
#include <MK60_FTM.h>
#include <MK60_gpio.h>
#include <MK60_uart.h>
#include <MK60_adc.h>
#include <libsc/com/uart_device.h>

int main(void)
{
	libsc::UartDevice uart(3, 115200);
	FTM_PWM_init(FTM0, FTM_CH0, 10000, 0);
	FTM_PWM_init(FTM0, FTM_CH1, 10000, 0);
	gpio_init(PTC0, GPO, 1);
	gpio_init(PTA10, GPO, 1);

	//FTM_QUAD_Init(FTM1);
	//FTM_QUAD_Init(FTM2);
	//int32_t encoderR=0, encoderL=0;
	//int32_t encoderRnew=0, encoderLnew=0;

	//adc_init(ADC0_SE8);
	adc_init(ADC0_SE9);
	//adc_init(ADC0_SE12);
	//adc_init(ADC0_SE13);
	//adc_init(ADC1_SE17);

	int16 gyro1=0,gyro2=0,gyro3=0,gyro4=0, sp=5850;
	int16 angle=0;
	uint32 motorspeed=0;
	float kp = 300;

	char buf[30];
	while(1){


		//encoderRnew = FTM_QUAD_get(FTM1) - encoderR;
		//encoderLnew = FTM_QUAD_get(FTM2) - encoderL;
		//FTM_QUAD_get(FTM1)=0;
		//FTM_QUAD_get(FTM2)=0;
		//gyro1 = adc_once (ADC0_SE8, ADC_16bit);
		if(adc_once (ADC0_SE9, ADC_16bit) > 20300)
			gyro2 = 20300;
		else if(adc_once (ADC0_SE9, ADC_16bit) < 8600)
			gyro2 = 8600;
		else
			gyro2 = adc_once (ADC0_SE9, ADC_16bit);
		gyro2 = gyro2 - 8600;
		angle = gyro2*90/11700;
		//angle = gyro2 - 5850; //45 degree
		//gyro3 = adc_once (ADC0_SE12, ADC_16bit);
		//gyro4 = adc_once (ADC0_SE13, ADC_16bit);
		//gyro2 = adc_once (ADC1_SE17, ADC_16bit); //21000 45 degree
		//encoderR = encoderRnew;
		//encoderL = encoderLnew;
		//snprintf(buf, 30, "g1:%d g2:%d g3:%d g4:%d 1:%d 2:%d\r\n", gyro1, gyro2, gyro3, gyro4, encoder, encoder2);




		if(angle==80){
			motorspeed = 0;

		}
		if(angle<80){
			motorspeed = (80-angle) * kp;
			gpio_set (PTC0, 0);
			gpio_set (PTA10, 0);
		}
		if(angle>80){
			motorspeed = (angle - 80) * kp;
			gpio_set (PTC0, 1);
			gpio_set (PTA10, 1);
		}



		motorspeed = (motorspeed>10000 ? 10000 : (motorspeed<0 ? 0 : motorspeed));


		//snprintf(buf, 30, "%d, %d\r\n", angle, motorspeed);
		//uart.SendStr(buf);

		FTM_PWM_Duty(FTM0, FTM_CH0, motorspeed);
		FTM_PWM_Duty(FTM0, FTM_CH1, motorspeed);

		//systick_delay_ms(1000);

	}


    return 0;
}
