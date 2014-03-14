#include <mini_common.h>
#include <hw_common.h>
#include <MK60_gpio.h>
#include <vectors.h>
#include <misc.h>

#include <libsc/com/lcd.h>
#include <libsc/com/bluetooth.h>
#include <libsc/com/k60/port_isr_manager.h>

using namespace libsc;


volatile int img[60][80];
volatile int start_drawing=0;

#define CAMERA_WIDTH 80
#define	CAMERA_HEIGHT 60
#define	INTERVAL 20

volatile int pit;
volatile int RowCnt=0;
volatile int HsCnt=0;

//__ISR void Pit0Handler()
//{
//	pit++;
//}


__ISR void pta6Handler()
{


	HsCnt++;

	if( (HsCnt%INTERVAL==0) && (RowCnt<CAMERA_HEIGHT) )
	{
		for(int x=0; x<80; x+=8, RowCnt++)
		{
			img[RowCnt][x+0] = gpio_get(PTC5);
			img[RowCnt][x+1] = gpio_get(PTC7);
			img[RowCnt][x+2] = gpio_get(PTC9);
			img[RowCnt][x+3] = gpio_get(PTC11);
			img[RowCnt][x+4] = gpio_get(PTC13);
			img[RowCnt][x+5] = gpio_get(PTC15);
			img[RowCnt][x+6] = gpio_get(PTC14);
			img[RowCnt][x+7] = gpio_get(PTD7);
		}
	}



	gpio_turn(PTE24);
}

__ISR void pta10Handler()
{
	RowCnt=0;
	start_drawing=1;
	gpio_turn(PTE25);
}

void ALL_INIT()
{
	for(int i=0; i<60; i++)
	{
		for(int j=0; j<80; j++)
		{
			img[i][j] = 0;
		}
	}

	//SetIsr(PIT0_VECTORn, Pit0Handler);

	DisableInterrupts;
	port_init(PTA6, IRQ_RISING | ALT1); 	//HREF
	gpio_ddr(PTA6, GPI);
	PortIsrManager *manager = PortIsrManager::GetInstance();
	manager->AddIsrHandler(PTA, PT6, pta6Handler);

	port_init(PTA10, IRQ_RISING | ALT1);
	gpio_ddr(PTA10, GPI);  					//VSYNC
	manager->AddIsrHandler(PTA, PT10, pta10Handler);
	EnableInterrupts;

	/* Camera Init */
	gpio_init(PTC5,  	GPI, 	0);
	gpio_init(PTC7,  	GPI, 	0);
	gpio_init(PTC9,  	GPI, 	0);
	gpio_init(PTC11,  	GPI, 	0);
	gpio_init(PTC13,  	GPI, 	0);
	gpio_init(PTC15, 	GPI, 	0);
	gpio_init(PTC14,  	GPI, 	0);
	gpio_init(PTD7, 	GPI, 	0);
	//gpio_init(PTA8, 	GPI, 	1);

	gpio_init(PTE24, GPO, 0);
	gpio_init(PTE25, GPO, 0);
	gpio_init(PTE26, GPO, 0);
}

int main(void)
{
	ALL_INIT();
	//UartDevice bluetooth(3, 115200);
	Lcd lcd(true);
	lcd.Clear(0x7777);

	while(1)
	{
		if(start_drawing)
		{

			for(int i=0,x=0; i<60; i++,x+=2)
			{
				for(int j=0,y=0; j<80; j++,y+=2)
				{
					//lcd.DrawPixelBuffer();
					lcd.DrawPixel(i, j, img[i][j] ? 0xFFFF : 0x0000);
					lcd.DrawPixel(i, j, img[i][j] ? 0xFFFF : 0x0000);
					lcd.DrawPixel(i, j, img[i][j] ? 0xFFFF : 0x0000);
					lcd.DrawPixel(i, j, img[i][j] ? 0xFFFF : 0x0000);
					//bluetooth.SendChar(img[j][i] ? '.' : '#');
				}
				//bluetooth.SendStr("\r\n");
			}

			start_drawing = 0;
			gpio_turn(PTE26);
		}


	}

	return 0;
}
