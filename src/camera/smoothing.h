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
		Smoothing(const int32_t output_period);
		~Smoothing();

		float SmoothingOutput();

		bool isFull();

		void SetOutputPeriod(const int32_t _output_period);
		void UpdateCurrentOutput(const float cur_output);

	private:
		int32_t output_period;
		int32_t output_count;
		float previous_output;
		float current_output;
		bool updated;
};

}



#endif /* SMOOTHING_H_ */
