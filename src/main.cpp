#include <mini_common.h>
#include <hw_common.h>

#include <MK60_gpio.h>
#include <vectors.h>

#include <libsc/com/bluetooth.h>
#include <libsc/com/lcd.h>
#include <libsc/com/led.h>
#include <libsc/com/ov7725.h>
#include <libsc/com/uart_device.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include <math.h>
#include <string.h>

using namespace libsc;

#define CAM_W 160
#define CAM_H 120
#define WHITE_BYTE 0xFF
#define BLACK_BYTE 0						//online RGB 565 calculator
#define RED_BYTE 0xF800						//http://www.henningkarlsen.com/electronics/calc_rgb565.php

namespace
{

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

}

class App
{
public:
    App();
    void Run();

private:
    void ShootOnceTest();
    void ShootContinuouslyTest();
    void DrawCenterPixel(const Byte* src, const int line);
    int GetPixel(const int x, const int y, const int offset);
    void PrintCenterLineEquation(float LineCenterX[]);

    Ov7725 m_cam;
    Lcd m_lcd;
    UartDevice bluetooth;
};

App::App()
        : m_cam(CAM_W, CAM_H), bluetooth(3, 115200)
{
    m_lcd.Clear(Lcd::GetRgb565(0x33, 0xB5, 0xE5));
}

void App::DrawCenterPixel(const Byte* src, const int line)
{/*
	int BlackCount = 0;
	int BlackSum = 0;
	float LineCenterX = -1;

	float LineCenterXSet[60];

	for(int i=1; i<=CAM_H; i++)
	{
		if(m_car.GetPixel(i*j-1)==BLACK)
		{
			BlackCount++;
			BlackSum+=j;
		}
		LineCenterX = (int) (BlackSum / BlackCount) ;
		LineCenterXSet[i] = LineCenterX;
	}

	return;
	*/
}

int App::GetPixel(const int x, const int y, const int offset)
{/*
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
    */
}

void App::PrintCenterLineEquation(float LineCenterXSet[])
{/*
	float s1=0; float s2=0; float s3=0; float s4=0;
	float slope;

	for(int i=0; i<CAM_H; i++)
	{
		s1+=LineCenterXSet[i]*i;
		s2+=LineCenterXSet[i];
		s3+=i;
		s4+=i*i;
	}

	slope = (CAM_H*s1 - s2*s3) / (CAM_H*s4 - s2*s2);					//from Excel equation for calculating the slope of giving n points

	float sumX=0; float sumY=0;
	float intercept;

	for(int i=0; i<CAM_H; i++)
	{
		sumX+=LineCenterXSet[i];
		sumY+=i;
	}

	intercept = (sumY/CAM_H) - (slope*sumX/CAM_H);

	bluetooth.SendStr(libutil::String::Format("y = %fx + %f\r\n", slope, intercept).c_str());
	*/
}

void App::Run()
{
    if (!m_cam.Init())
    {
        while (true)
        {
            LOG_E("Failed while initializing the camera");
            DELAY_MS(250);
        }
    }

    ShootOnceTest();
    //ShootContinuouslyTest();
}

void App::ShootOnceTest()
{
    libutil::Clock::ClockInt prev_time = libutil::Clock::Time();
    int frame_count = 0;
    m_cam.ShootOnce();
    while (true)
    {
        while (!m_cam.IsImageReady())
        {}

        const Byte *src = m_cam.LockBuffer();
        //uart.SendChar(0);
        //uart.SendChar(255);
        //uart.SendChar(1);
        //uart.SendChar(0);
        for (int i = CAM_H - 1; i >= 0; --i)
        {
            const Byte *buf = ExpandPixel(src, i);
            //uart.SendBuffer(buf, CAM_W);

            m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
            //uart.SendChar('\n');
            //delete[] buf;
        }
/*
        for(int i=0; i<120; i++)
        {
        	DrawCenterPixel(src, i);
        }
*/
       // printCenterLineEquation(LineCenterXSet);

        m_cam.UnlockBuffer();
        ++frame_count;
        //uart.SendStr("\n\n\n");

        if (libutil::Clock::TimeDiff(libutil::Clock::Time(), prev_time) >= 1000)
        {
            prev_time = libutil::Clock::Time();
            printf("FPS: %d\n", frame_count);
            frame_count = 0;
        }

        m_cam.ShootOnce();
    }
}

void App::ShootContinuouslyTest()
{
    libutil::Clock::ClockInt prev_time = libutil::Clock::Time();
    int frame_count = 0;
    m_cam.ShootContinuously();
    while (true)
    {
        const Byte *src = m_cam.LockBuffer();
        for (int i = CAM_H - 1; i >= 0; --i)
        {
            const Byte *buf = ExpandPixel(src, i);
            m_lcd.DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
        }
        m_cam.UnlockBuffer();
        ++frame_count;

        if (libutil::Clock::TimeDiff(libutil::Clock::Time(), prev_time) >= 1000)
        {
            prev_time = libutil::Clock::Time();
            printf("FPS: %d\n", frame_count);
            frame_count = 0;
        }

        DELAY_MS(16);
    }
}

int main()
{
    libutil::Clock::Init();
    UartDevice uart(3, 115200);
    libutil::InitDefaultFwriteHandler(&uart);

/*
    Led led0(0);
    Led led1(1);
    Led led2(2);
    Led led3(3);

    while(1)
    {
    	led0.SetEnable(1);
    	DELAY_MS(50);
    	led1.SetEnable(1);
    	DELAY_MS(50);
    	led2.SetEnable(1);
    	DELAY_MS(50);
    	led3.SetEnable(1);
    	DELAY_MS(50);

    	led0.SetEnable(0);
    	DELAY_MS(50);
    	led1.SetEnable(0);
    	DELAY_MS(50);
    	led2.SetEnable(0);
    	DELAY_MS(50);
    	led3.SetEnable(0);
    	DELAY_MS(50);
    }
*/
    App app;
    app.Run();
    return 0;
}
