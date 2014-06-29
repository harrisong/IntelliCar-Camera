/*
 * smoothing.h
 *
 *  Created on: 2014年6月19日
 *      Author: FaiFai
 */

#ifndef SMOOTHING_H_
#define SMOOTHING_H_

#include <cstdint>

namespace camera
{

class Smoothing
{

	public:
		Smoothing(const int output_period);
		~Smoothing();

		int32_t SmoothingOutput();

		void UpdateCurrentOutput(const int32_t cur_output);

	private:
		int output_period;
		int output_count;
		float previous_output;
		float current_output;
};

}



#endif /* SMOOTHING_H_ */
