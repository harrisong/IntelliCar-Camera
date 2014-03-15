#include <cstdint>
#include <mini_common.h>
#include <hw_common.h>
#include <init.h>
/*#include <libsc/com/uart_device.h>
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>
using namespace libsc;
using namespace libutil;
libsc::UartDevice uart(3, 115200);
//uart.StartReceive();
libutil::TunableIntManager<1> manager(&uart);
const libutil::TunableInt *kp = manager.Register("kp", 0);

manager.Start();*/

int main(void)
{
	init();
    return 0;
}


