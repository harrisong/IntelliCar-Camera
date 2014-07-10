#include <hw_common.h>
#include <MK60_gpio.h>
#include <camera/camera_app.h>

int main()
{
	gpio_init(PTB22, GPO, 1);
	DELAY_MS(100);
	gpio_set(PTB22, 0);

	camera::CameraApp app;
	app.Run();

    return 0;
}
