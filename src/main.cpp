#include <mini_common.h>
#include <hw_common.h>
#include <MK60_gpio.h>
#include <vectors.h>
#include <misc.h>

#include <libsc/com/lcd.h>
#include <libsc/com/bluetooth.h>
#include <libsc/com/k60/port_isr_manager.h>

using namespace libsc;

volatile int img[60][80];		//Image array
volatile int start_drawing=0;

#define CAMERA_WIDTH 80			//Camera Resolution
#define	CAMERA_HEIGHT 60
#define	INTERVAL 20

//volatile int pit;
volatile int RowCnt=0;
volatile int HsCnt=0;

//__ISR void Pit0Handler()
//{
//	pit++;
//}


__ISR void pta6Handler()
{
	HsCnt++;				//HREF counter

	if( (HsCnt%INTERVAL==0) && (RowCnt<CAMERA_HEIGHT) )
	{
		for(int x=0; x<80; x+=8, RowCnt++)					//Store a row of pixels
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

	gpio_turn(PTE24);		//LED indicates HREF happened
}

__ISR void pta10Handler()
{
	RowCnt=0;				//Reset
	start_drawing=1;		//Start Drawing
	gpio_turn(PTE25);		//LED indicates VSYNC happened
}

void ALL_INIT()
{

	/* Image Array Init */
	for(int i=0; i<60; i++)
	{
		for(int j=0; j<80; j++)
		{
			img[i][j] = 0;
		}
	}
	/* Image Array Init */

	/*	Interrupts Init	*/
	DisableInterrupts;
	port_init(PTA6, IRQ_RISING | ALT1); 	//HREF: Interrupts when a row of pixels is received.
	gpio_ddr(PTA6, GPI);
	PortIsrManager *manager = PortIsrManager::GetInstance();
	manager->AddIsrHandler(PTA, PT6, pta6Handler);

	port_init(PTA10, IRQ_RISING | ALT1);
	gpio_ddr(PTA10, GPI);  					//VSYNC: Interrupts when the whole picture is received.
	manager->AddIsrHandler(PTA, PT10, pta10Handler);
	EnableInterrupts;
	/*	Interrupts Init */

	/* Camera Pixel Pins Init */
	gpio_init(PTC5,  	GPI, 	0);
	gpio_init(PTC7,  	GPI, 	0);
	gpio_init(PTC9,  	GPI, 	0);
	gpio_init(PTC11,  	GPI, 	0);
	gpio_init(PTC13,  	GPI, 	0);
	gpio_init(PTC15, 	GPI, 	0);
	gpio_init(PTC14,  	GPI, 	0);
	gpio_init(PTD7, 	GPI, 	0);
	/* Camera Pixel Pins Init */

	//gpio_init(PTA8, 	GPI, 	1);

	/* Onboard LED Init	*/
	gpio_init(PTE24, GPO, 0);
	gpio_init(PTE25, GPO, 0);
	gpio_init(PTE26, GPO, 0);
	gpio_init(PTE27, GPO, 0);
	/* Onboard LED Init	*/
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
					lcd.DrawPixel(i, j, img[i][j] ? 0x0000 : 0xFFFF);			//Enlarge a pixel to 4 pixels in LCD
					lcd.DrawPixel(i, j, img[i][j] ? 0x0000 : 0xFFFF);
					lcd.DrawPixel(i, j, img[i][j] ? 0x0000 : 0xFFFF);
					lcd.DrawPixel(i, j, img[i][j] ? 0x0000 : 0xFFFF);
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
