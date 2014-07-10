/*
 * lcdmenu.h
 *
 *  Created on: 2014年6月18日
 *      Author: FaiFai
 */

#ifndef LCDMENU_H_
#define LCDMENU_H_

#include "helper.h"
#include <cstdint>

namespace camera
{

class LcdMenu
{
	public:
		static constexpr uint8_t W = 128;
		static constexpr uint8_t H = 160;
		static constexpr uint8_t FONT_W = 8;
		static constexpr uint8_t FONT_H = 16;

		LcdMenu(Car* car, char* title, char** choices, int num_choice);
		~LcdMenu();

		int GetSelectedChoice() const;
		char* GetChoice(const int line) const;
		void SetChoice(const int line, char* choice);

		void CreateMenu();
		void WaitForSelection();

	private:
		Car* car_pt;
		Helper helper;
		char* title;
		char** choices;
		int num_choice;
		int selected_choice;
		bool moved;
};

}



#endif /* LCDMENU_H_ */
