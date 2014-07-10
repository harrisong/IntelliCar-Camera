/*
 * smoothing.cpp
 *
 *  Created on: 2014年6月19日
 *      Author: FaiFai
 */
//

#include "smoothing.h"

namespace camera
{

Smoothing::Smoothing(const int32_t output_period)
	: output_period(output_period), output_count(0), previous_output(0), current_output(0)
{

}

Smoothing::~Smoothing()
{

}

void Smoothing::SetOutputPeriod(const int32_t _output_period)
{
	output_period = _output_period;
}

void Smoothing::UpdateCurrentOutput(const float cur_output)
{
	output_count = 0;
	previous_output = current_output;
	current_output = cur_output;
	updated = true;
}

bool Smoothing::isFull() {
	float diff = previous_output-current_output;
	diff = diff>=0 ? diff : -diff;
	return (diff < 0.05) && (int) previous_output!=0 && (int) current_output!=0;
}

float Smoothing::SmoothingOutput()
{
	if(updated) {
		output_count++;
	}

	if(!isFull())
	{
		float smoothing_output = ((current_output - previous_output) * output_count) / output_period + previous_output;

		if(output_count>=output_period) {
			output_count = 0;
			updated = false;
			previous_output = current_output;
		}

		return smoothing_output;
	}

	return current_output;
}

}
