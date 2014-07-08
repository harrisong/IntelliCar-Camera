/*
 * lcdmenu.cpp
 *
 *  Created on: 2014年6月18日
 *      Author: FaiFai
 */

#include "hw_common.h"
#include "lcdmenu.h"
#include <libsc/com/joystick.h>
#include <libsc/com/button.h>
#include <libutil/string.h>


namespace camera
{

LcdMenu::LcdMenu(Car* car, char* title, char** choices, int num_choice)
	: car_pt(car), helper(car), title(title), choices(choices), num_choice(num_choice), selected_choice(-1)
{}

LcdMenu::~LcdMenu()
{

}

int LcdMenu::GetSelectedChoice() const
{
	return selected_choice;
}

char* LcdMenu::GetChoice(const int line) const
{
	return choices[line-1];
}

void LcdMenu::SetChoice(const int line, char* choice)
{
	choices[line-1] = choice;
}

void LcdMenu::WaitForSelection()
{
	libsc::Joystick* m_joystick = car_pt->GetJoystick();

	int maxh = H/FONT_H - 1;

	int ptr_pos = 1;
	int ptr_output_pos = 1;
	int viewport[2] = {1, maxh};

	uint8_t mx;

	uint32_t pt = libutil::Clock::Time();

	while(selected_choice==-1){

//		if(libutil::Clock::TimeDiff(libutil::Clock::Time(), pt) > 3000)
//		{
//			selected_choice = 1;
//			break;
//		}

		mx = FONT_W * 5;
		helper.Printline( mx , 0, libutil::String::Format("%02d/%02d", ptr_pos, num_choice).c_str());
		mx = FONT_W * 11;
		helper.Printline( mx , 0, libutil::String::Format("%.2fV", car_pt->GetVolt()).c_str());
		switch (m_joystick->GetState())
		{
			case libsc::Joystick::DOWN:

				if(ptr_pos + 1 <= num_choice){
					++ptr_pos;
					++ptr_output_pos;
					ptr_output_pos = ptr_output_pos > maxh ? maxh : ptr_output_pos;

					if(ptr_pos > viewport[1]){
						viewport[0]++;
						viewport[1]++;
						for(int i=1; i<=maxh; i++) {
							if(ptr_output_pos == i) helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1], true);
							else helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1]);
						}
					}

				}else{
					ptr_pos = 1;
					ptr_output_pos = 1;
					viewport[0] = 1;
					viewport[1] = maxh;
					for(int i=1; i<=maxh; i++) {
						if(ptr_output_pos == i) helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1], true);
						else helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1]);
					}
				}
				if(ptr_output_pos != 1) helper.Printline(FONT_H * (ptr_output_pos-1), choices[viewport[0]-1+ptr_output_pos-1-1]);
				helper.Printline(FONT_H * ptr_output_pos, choices[viewport[0]-1+ptr_output_pos-1], true);
				DELAY_MS(100);
				break;

			case libsc::Joystick::UP:
				if(ptr_pos - 1 >= 1){
					--ptr_pos;
					--ptr_output_pos;
					ptr_output_pos = ptr_output_pos < 1 ? 1 : ptr_output_pos;

					if(ptr_pos < viewport[0]){
						viewport[0]--;
						viewport[1]--;
						for(int i=1; i<=maxh; i++) {
							//helper.Printline(FONT_H * i, "                ");
							if(ptr_output_pos == i) helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1], true);
							else helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1]);
						}
					}

				}else{
					ptr_pos = num_choice;
					ptr_output_pos = maxh;
					viewport[0] = num_choice - maxh + 1;
					viewport[1] = num_choice;
					for(int i=1; i<=maxh; i++) {
						//helper.Printline(FONT_H * i, "                ");
						if(ptr_output_pos == i) helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1], true);
						else helper.Printline(FONT_H * i, choices[viewport[0]-1+i-1]);
					}
				}
				if(ptr_output_pos != maxh) helper.Printline(FONT_H * (ptr_output_pos+1), choices[viewport[0]-1+ptr_output_pos-1+1]);
				helper.Printline(FONT_H * ptr_output_pos, choices[viewport[0]-1+ptr_output_pos-1], true);
				//PrintPtr(ptr_output_pos * FONT_H);
				DELAY_MS(100);

				break;

			case libsc::Joystick::SELECT:
				selected_choice = ptr_pos;
				break;
		}
	}

}

void LcdMenu::CreateMenu()
{
	int maxh = H/FONT_H - 1;

	helper.Printline(FONT_H * 0, title);
	helper.Printline(FONT_H * 1, choices[0], true);

	for(int i=1; i<=maxh; i++) {
		helper.Printline(FONT_H * (i+1), choices[i]);
	}

}

}
