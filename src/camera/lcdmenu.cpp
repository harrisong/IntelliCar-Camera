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
#include <MK60_gpio.h>

namespace camera
{

LcdMenu::LcdMenu(Car* car, char* title, char** choices, int num_choice)
	: car_pt(car), helper(car), title(title), choices(choices), num_choice(num_choice), selected_choice(-1), moved(false)
{
	m_instance = this;
	maxh = H/FONT_H - 1;
	ptr_pos = 1;
	ptr_output_pos = 1;
	viewport[0] = 1;
	viewport[1] = maxh;
}

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

__ISR void LcdMenu::MoveMenu(){
//	libsc::Joystick* m_joystick = car_pt->GetJoystick();

	uint8_t n = 0;
	n=5;
	if(PORTC_ISFR & (1<<n)){
			PORTC_ISFR  |= (1<<n);
			printf("5\n");
			m_instance->moved = true;
			if(m_instance->ptr_pos + 1 <= m_instance->num_choice){
				++m_instance->ptr_pos;
				++m_instance->ptr_output_pos;
				m_instance->ptr_output_pos = m_instance->ptr_output_pos > m_instance->maxh ? m_instance->maxh : m_instance->ptr_output_pos;

				if(m_instance->ptr_pos > m_instance->viewport[1]){
					m_instance->viewport[0]++;
					m_instance->viewport[1]++;
					for(int i=1; i<=m_instance->maxh; i++) {
						if(m_instance->ptr_output_pos == i) m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1], true);
						else m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1]);
					}
				}

			}else{
				m_instance->ptr_pos = 1;
				m_instance->ptr_output_pos = 1;
				m_instance->viewport[0] = 1;
				m_instance->viewport[1] = m_instance->maxh;
				for(int i=1; i<=m_instance->maxh; i++) {
					if(m_instance->ptr_output_pos == i) m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1], true);
					else m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1]);
				}
			}
			if(m_instance->ptr_output_pos != 1) m_instance->helper.Printline(FONT_H * (m_instance->ptr_output_pos-1), m_instance->choices[m_instance->viewport[0]-1+m_instance->ptr_output_pos-1-1]);
			m_instance->helper.Printline(FONT_H * m_instance->ptr_output_pos, m_instance->choices[m_instance->viewport[0]-1+m_instance->ptr_output_pos-1], true);
//			DELAY_MS(100);
	}

	n=6;
	if(PORTC_ISFR & (1<<n)){
			PORTC_ISFR  |= (1<<n);
			printf("6\n");
			m_instance->moved = true;
			if(m_instance->ptr_pos - 1 >= 1){
				--m_instance->ptr_pos;
				--m_instance->m_instance->ptr_output_pos;
				m_instance->ptr_output_pos = m_instance->ptr_output_pos < 1 ? 1 : m_instance->ptr_output_pos;

				if(m_instance->ptr_pos < m_instance->viewport[0]){
					m_instance->viewport[0]--;
					m_instance->viewport[1]--;
					for(int i=1; i<=m_instance->maxh; i++) {
						//helper.Printline(FONT_H * i, "                ");
						if(m_instance->ptr_output_pos == i) m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1], true);
						else m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1]);
					}
				}

			}else{
				m_instance->ptr_pos = m_instance->num_choice;
				m_instance->ptr_output_pos = m_instance->maxh;
				m_instance->viewport[0] = m_instance->num_choice - m_instance->maxh + 1;
				m_instance->viewport[1] = m_instance->num_choice;
				for(int i=1; i<=m_instance->maxh; i++) {
					//helper.Printline(FONT_H * i, "                ");
					if(m_instance->ptr_output_pos == i) m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1], true);
					else m_instance->helper.Printline(FONT_H * i, m_instance->choices[m_instance->viewport[0]-1+i-1]);
				}
			}
			if(m_instance->ptr_output_pos != m_instance->maxh) m_instance->helper.Printline(FONT_H * (m_instance->ptr_output_pos+1), m_instance->choices[m_instance->viewport[0]-1+m_instance->ptr_output_pos-1+1]);
			m_instance->helper.Printline(FONT_H * m_instance->ptr_output_pos, m_instance->choices[m_instance->viewport[0]-1+m_instance->ptr_output_pos-1], true);
			//PrintPtr(m_instance->ptr_output_pos * FONT_H);
//			DELAY_MS(100);

	}

	n=9;
	if(PORTC_ISFR & (1<<n)){
			PORTC_ISFR  |= (1<<n);
			printf("9\n");
			m_instance->moved = true;
			m_instance->selected_choice = m_instance->ptr_pos;
			DisableIsr(PORTC_VECTORn);
	}

	m_instance->mx = FONT_W * 5;
	m_instance->helper.Printline( m_instance->mx , 0, libutil::String::Format("%02d/%02d", m_instance->ptr_pos, m_instance->num_choice).c_str());

}

void LcdMenu::WaitForSelection()
{

	port_init(PTC5, ALT1 | IRQ_FALLING );
	port_init(PTC6, ALT1 | IRQ_FALLING );
	port_init(PTC9, ALT1 | IRQ_FALLING );

	SetIsr(PORTC_VECTORn, MoveMenu);
	EnableIsr(PORTC_VECTORn);

	uint32_t pt = libutil::Clock::Time();

//	while(selected_choice==-1){
//		if(libutil::Clock::TimeDiff(libutil::Clock::Time(), pt) > 2500 && !moved )
//		{
//			selected_choice = 1;
//			break;
//		}
//	}
	mx = FONT_W * 5;
	helper.Printline( mx , 0, libutil::String::Format("%02d/%02d", ptr_pos, num_choice).c_str());

	mx = FONT_W * 11;
	helper.Printline( mx , 0, libutil::String::Format("%.2fV", car_pt->GetVolt()).c_str());

	while(selected_choice==-1);

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

LcdMenu *LcdMenu::m_instance = nullptr;

}
