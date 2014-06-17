/*
 * helper.cpp
 *
 *  Created on: 2014年6月18日
 *      Author: FaiFai
 */

#include "helper.h"
#include <cstring>
#include <libutil/string.h>

namespace camera
{

Helper::Helper(Car* m_car)
{
	car_pt = m_car;
}

int Helper::Clamp(const int x, const int lowerBound, const int upperBound)
{
	if(x<lowerBound)
		return lowerBound;
	else if(x>upperBound)
		return upperBound;
	else
		return x;
}

int Helper::GetPixel(const Byte* src, const int x, const int y)
{
    const int offset = x/8 + (y * CAM_W / 8);

    return (src[offset] >> (x%8) & 0x01) ? BLACK_BYTE : WHITE_BYTE;
}

Byte* Helper::ExpandPixel(const Byte *src, const int line)
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

void Helper::Printline(uint8_t* x, uint8_t y, const char* s, const uint16_t TXT_COLOR, const uint16_t BG_COLOR){

	while(*s){
		if(y==0) car_pt->GetLcd()->DrawChar(*x, y, *s, WHITE, BLACK);
		else car_pt->GetLcd()->DrawChar(*x, y, *s, TXT_COLOR, BG_COLOR);
		*x+=car_pt->GetLcd()->FONT_W;
		s++;
	}

}

void Helper::Printline(uint8_t y, const char* s, bool inverted){
	uint8_t x=0;
	Printline(&x, y, s, WHITE, INVERTED_BG_COLOR);
	for(int i=x; i<car_pt->GetLcd()->W; i++) car_pt->GetLcd()->DrawChar(i, y, ' ', WHITE, INVERTED_BG_COLOR);

}

void Helper::Printline(uint8_t y, const char* s){
	uint8_t x=0;
	if(y==0){
		for(int i=0; i<x; i++) car_pt->GetLcd()->DrawChar(i, y, ' ', WHITE, BLACK);
	}else{
		for(int i=0; i<x; i++) car_pt->GetLcd()->DrawChar(i, y, ' ', BLACK, WHITE);
	}
	Printline(&x, y, s);
	if(y==0){
		for(int i=x; i<car_pt->GetLcd()->W; i++) car_pt->GetLcd()->DrawChar(i, y, ' ', WHITE, BLACK);
	}else{
		for(int i=x; i<car_pt->GetLcd()->W; i++) car_pt->GetLcd()->DrawChar(i, y, ' ', BLACK, WHITE);
	}
}

void Helper::PrintPtr(uint8_t y){
	for(int i=car_pt->GetLcd()->FONT_H+1; i<car_pt->GetLcd()->H; i++) car_pt->GetLcd()->DrawChar(0, i, ' ', WHITE, WHITE);
	car_pt->GetLcd()->DrawChar(0, y, '>', 0, WHITE);
}

void Helper::PrintCam(){

	const Byte* src = car_pt->GetCamera()->LockBuffer();

	for (int i = CAM_H - 1; i >= 0; --i)
	{
		const Byte *buf = ExpandPixel(src, i);
		car_pt->GetLcd()->DrawGrayscalePixelBuffer((CAM_H - 1) - i, 0, 1, CAM_W, buf);
	}

	//---------------------Edge Detection---------------------//
	int LeftEdgeX, CenterX[CAM_H], RightEdgeX;
	memset(CenterX, -1, CAM_H);

	for(int y=CAM_H-1; y>=0; y--)
	{
		LeftEdgeX = -1;
		RightEdgeX = -1;

		if(y+2>=CAM_H)
			CenterX[y] = CenterX[y+1] + (CenterX[y+1] - CenterX[y+2]);
		else
			CenterX[y] = CAM_W/2;

		CenterX[y] = Clamp(CenterX[y], 0, CAM_W-1);

		int x = CenterX[y];

		if(
			GetPixel(src, x, y)==BLACK_BYTE
			&& GetPixel(src, Clamp(x-1, 0, CAM_W-1), y)==BLACK_BYTE
			&& GetPixel(src, Clamp(x+1, 0, CAM_W-1), y)==BLACK_BYTE
		)
		{
			const char* s = libutil::String::Format("%d, %d, %d", x, x-1, x+1).c_str();
			Printline(car_pt->GetLcd()->FONT_H * 7+30, s);
			break;
		}

		for(int x=CenterX[y]; x>=0; x--)
		{
			if(
				x-1>=0 && x-2>=0 && x-3>=0
				&& GetPixel(src, x, y) == WHITE_BYTE
				&& GetPixel(src, x-1, y) == BLACK_BYTE
				&& GetPixel(src, x-2, y) == BLACK_BYTE
				&& GetPixel(src, x-3, y) == BLACK_BYTE
			)
			{
				LeftEdgeX = x-1;
				break;
			}
		}

		for(int x=CenterX[y]; x<CAM_W; x++)
		{
			if(
				x+1<CAM_W && x+2<CAM_W && x+3<CAM_W
				&& GetPixel(src, x, y) == WHITE_BYTE
				&& GetPixel(src, x+1, y) == BLACK_BYTE
				&& GetPixel(src, x+2, y) == BLACK_BYTE
				&& GetPixel(src, x+3, y) == BLACK_BYTE
			)
			{
				RightEdgeX = x+1;
				break;
			}
		}

		if(LeftEdgeX==-1 || RightEdgeX==-1)
		{
			if(LeftEdgeX==-1)
				LeftEdgeX = 0;
			if(RightEdgeX==-1)
				RightEdgeX = CAM_W-1;

		}

		CenterX[y] = (int) round((LeftEdgeX + RightEdgeX)/2);

		car_pt->GetLcd()->DrawPixel((CAM_H - 1) - y, LeftEdgeX, libutil::GetRgb565(0xFF, 0x00, 0x00));
		car_pt->GetLcd()->DrawPixel((CAM_H - 1) - y, RightEdgeX, libutil::GetRgb565(0x00, 0xFF, 0x00));
		car_pt->GetLcd()->DrawPixel((CAM_H - 1) - y, CenterX[y], libutil::GetRgb565(0x00, 0x00, 0xFF));
	}
	//---------------------Edge Detection---------------------//


	car_pt->GetCamera()->UnlockBuffer();
	uint16_t sec = libutil::Clock::Time()/1000;
	const char* s = libutil::String::Format("Time: %02d",sec).c_str();
	Printline(car_pt->GetLcd()->FONT_H * 7, s);
	//DELAY_MS(10);
}

}

