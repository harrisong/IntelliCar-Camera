/*
 * helper.h
 *
 *  Created on: 2014年6月18日
 *      Author: FaiFai
 */

#ifndef HELPER_H_
#define HELPER_H_

#include "vars.h"
#include "car.h"
#include <libutil/string.h>

namespace camera
{

class Helper
{
	public:
		Helper(Car* car);
		~Helper();

		int abs(const int x);
		int Clamp(const int x, const int lowerBound, const int upperBound);
		int GetPixel(const Byte* src, const int x, const int y);
		Byte* ExpandPixel(const Byte *src, const int line);

		void Printline(uint8_t y, const char* s);
		void Printline(uint8_t y, const char* s, bool);
		void Printline(uint8_t* x, uint8_t y, const char* s, const uint16_t TXT_COLOR = BLACK, const uint16_t BG_COLOR = WHITE);
		void Printline(uint8_t x, uint8_t y, const char* s){
			Printline(&x, y, s);
		}

		void PrintPtr(uint8_t y);

		void PrintCam();

		void CreateMenu(const char** menu, const int num_choice);

	private:
		Car* car_pt;
};

}



#endif /* HELPER_H_ */
