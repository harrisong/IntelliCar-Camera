#include <mini_common.h>
#include <hw_common.h>
#include <MK60_gpio.h>
#include <vectors.h>
#include <misc.h>

#include <libsc/com/lcd.h>
#include <libsc/com/bluetooth.h>
#include <libsc/com/k60/port_isr_manager.h>

using namespace libsc;

volatile int y=0;
volatile int img[80][60];
volatile int start_drawing=0;

volatile int pit_haha = 0;

__ISR void Pit0Handler()
{
	++pit_haha;
}


__ISR void pta6Handler()
{
	DisableInterrupts;

	for(int x=0; x<80; x+=8)
	{
		img[x+0][y] = gpio_get(PTC5);
		img[x+1][y] = gpio_get(PTC7);
		img[x+2][y] = gpio_get(PTC9);
		img[x+3][y] = gpio_get(PTC11);
		img[x+4][y] = gpio_get(PTC13);
		img[x+5][y] = gpio_get(PTC15);
		img[x+6][y] = gpio_get(PTC14);
		img[x+7][y] = gpio_get(PTD7);
	}

	y++;

	if(y>=60)
	{
		y = 0;
		start_drawing = 1;
	}

	EnableInterrupts;
	//gpio_turn(PTE24);
}

__ISR void pta10Handler()
{
	//gpio_turn(PTE24);
}

void ALL_INIT()
{
	for(int i=0; i<80; i++)
	{
		for(int j=0; j<60; j++)
		{
			img[i][j] = 0;
		}
	}
	//HAHA
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
	gpio_init(PTA8, 	GPO, 	1);

	gpio_init(PTE24, GPO, 0);

}

int main(void)
{
	ALL_INIT();
	//UartDevice bluetooth(3, 115200);
	Lcd lcd(true);
	lcd.Clear(0x7777);

	while(1)
	{
		//lcd.DrawPixelBuffer(0, 0, 80, 60, 0xFFFF, 0x0000, img);
		for(int i=0,x=0; i<60 && start_drawing; i++,x+=2)
		{
			for(int j=0,y=0; j<80; j++,y+=2)
			{
				lcd.DrawPixel(119-x, y, img[j][i] ? 0x2222 : 0xBBBB);
				lcd.DrawPixel(119-x, y+1, img[j][i] ? 0x2222 : 0xBBBB);
				lcd.DrawPixel(119-x-1, y, img[j][i] ? 0x2222 : 0xBBBB);
				lcd.DrawPixel(119-x-1, y+1, img[j][i] ? 0x2222 : 0xBBBB);
				//bluetooth.SendChar(img[j][i] ? '.' : '#');
			}
			//bluetooth.SendStr("\r\n");
		}

		start_drawing = 0;
	}

	return 0;
}
