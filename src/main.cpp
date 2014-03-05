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
	//FTM_PWM_init(FTM0, FTM_CH0, 10000, 1000);
	//FTM_PWM_init(FTM0, FTM_CH1, 10000, 1000);
	//gpio_init(PTC0, GPO, 1);
	//gpio_init(PTB17, GPO, 1);

	//FTM_QUAD_Init(FTM1);
	//FTM_QUAD_Init(FTM2);
	int32_t encoderR=0, encoderL=0;
	int32_t encoderRnew=0, encoderLnew=0;

	adc_init(ADC1_SE4b);
	adc_init(ADC1_SE5b);
	adc_init(ADC0_SE14);
	adc_init(ADC1_SE7b);

	uint16 gyro1=0,gyro2=0,gyro3=0,gyro4=0;

	char buf[30];
	while(1){
		//FTM_PWM_Duty(FTM0, FTM_CH0, 2500);
		//FTM_PWM_Duty(FTM0, FTM_CH1, 2500);

		//encoderRnew = FTM_QUAD_get(FTM1) - encoderR;
		//encoderLnew = FTM_QUAD_get(FTM2) - encoderL;
		//FTM_QUAD_get(FTM1)=0;
		//FTM_QUAD_get(FTM2)=0;
		//gyro1 = adc_once (ADC1_SE4b, ADC_16bit);
		//gyro2 = adc_once (ADC1_SE5b, ADC_16bit);
		gyro3 = adc_once (ADC0_SE14, ADC_16bit);
		//gyro4 = adc_once (ADC1_SE7b, ADC_16bit);

		//encoderR = encoderRnew;
		//encoderL = encoderLnew;
		//snprintf(buf, 30, "g1:%d g2:%d g3:%d g4:%d 1:%d 2:%d\r\n", gyro1, gyro2, gyro3, gyro4, encoder, encoder2);
		snprintf(buf, 30, "%d, %d, %d, %d\r\n", gyro1, gyro2, gyro3, gyro4);

		uart.SendStr(buf);

		systick_delay_ms(500);
	}


    return 0;
}
