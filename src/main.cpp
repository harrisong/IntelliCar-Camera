#include <hw_common.h>
#include <libsc/com/motor.h>
#include <camera/camera_app.h>
#include <MK60_FTM.h>
#include <MK60_gpio.h>

int main()
{
//	gpio_init(PTC9, GPI, 1);
//	while(gpio_get(PTC9)==1);
//
//	gpio_init(PTE6, GPO, 1);
//	gpio_init(PTE7, GPO, 1);
//
//	FTM_PWM_init(FTM0, FTM_CH6, 1000, 9000);
//	FTM_PWM_init(FTM0, FTM_CH7, 1000, 9000);
//
//	gpio_init(PTD0, GPO, 1);
//	gpio_init(PTD1, GPO, 1);
//
//
//	while(true){
//		gpio_turn(PTD0);
//		gpio_turn(PTD1);
//		DELAY_MS(100);
//	}

	camera::CameraApp app;
	app.Run();

    return 0;
}
