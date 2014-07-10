/*
 * smoothing.cpp
 *
 *  Created on: 2014年6月19日
 *      Author: FaiFai
 */


#include "smoothing.h"

namespace camera
{

Smoothing::Smoothing(const int32_t output_period)
	: output_period(output_period), output_count(0), previous_output(-1), current_output(0)
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

float Smoothing::SmoothingOutput()
{
	if(updated) {
		output_count++;
	}

	float smoothing_output = ((current_output - previous_output) * output_count) / output_period + previous_output;

	if(output_count>=output_period) {
		output_count = 0;
		updated = false;
	}

	return smoothing_output;
}

}
