#include "camera/camera_app.h"

#include <libsc/com/motor.h>
using namespace libsc;

int main()
{
	camera::CameraApp app;
    app.Run();

	/*Motor m1(0), m2(1);
	m1.SetClockwise(true);
	m2.SetClockwise(true);

	while(true){
		m1.SetPower(5000);
		m2.SetPower(5000);
	}*/

    return 0;
}
