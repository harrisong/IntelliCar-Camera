#include <hw_common.h>
#include <MK60_gpio.h>
#include <camera/camera_app.h>

int main()
{
	camera::CameraApp app;
	app.Run();

    return 0;
}
