#include <mini_common.h>
#include <hw_common.h>
#include "camera/camera_app.h"
#include <MK60_gpio.h>

int main()
{
    camera::CameraApp app;
    app.Run();


    return 0;
}
