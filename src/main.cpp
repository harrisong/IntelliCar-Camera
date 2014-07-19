#include <hw_common.h>
#include <MK60_gpio.h>
#include <camera/camera_app.h>

camera::CameraApp* app;

int main()
{
//	gpio_init(PTB22, GPO, 1);
//	DELAY_MS(100);
//	gpio_set(PTB22, 0);

	app = new camera::CameraApp;
	app->Run();

    return 0;
}
