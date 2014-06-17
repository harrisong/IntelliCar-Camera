#include <mini_common.h>
#include <hw_common.h>
#include "camera/camera_app.h"
#include <MK60_gpio.h>
#include <MK60_uart.h>
#include <libsc/com/uart_device.h>
#include <libsc/com/lcd.h>
using namespace libsc;
/*
Byte* ExpandPixel(const Byte *src, const int line)
{
    static Byte product[CAM_W];
    Byte *it = product;
    const int offset = line * CAM_W / 8;
    for (int i = 0; i < CAM_W / 8; ++i)
    {
        *(it++) = ((src[i + offset] >> 7) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 6) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 5) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 4) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 3) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 2) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 1) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
        *(it++) = ((src[i + offset] >> 0) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
    }
    return product;
}
*/

int main()
{
	/*libsc::Lcd m_lcd;
	m_lcd.Clear(WHITE);
	//uart_init(UART1, 115200);
	libsc::UartDevice m_uart(1, 115200);
	libutil::InitDefaultFwriteHandler(&m_uart);
	libsc::Ov7725 m_cam(CAM_W, CAM_H);
	m_cam.Init();
	m_cam.ShootContinuously();



	while(true){
		const Byte* src = m_cam.LockBuffer();
		//uart_putstr(UART1,"FUCKFUCKFUCK\n");
		printf("FUCKFUCKFUCK\n");
		for (int i = CAM_H - 1; i >= 0; --i)
		{
			const Byte *buf = ExpandPixel(src, i);
			m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);

		}
		m_cam.UnlockBuffer();
		DELAY_MS(100);
	}*/
	camera::CameraApp app;
    app.Run();

    return 0;
}
