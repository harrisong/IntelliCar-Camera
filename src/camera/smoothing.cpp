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
	: output_period(output_period), output_count(0), previous_output(0), current_output(0)
{

}

Smoothing::~Smoothing()
{

}

void Smoothing::UpdateCurrentOutput(const float cur_output)
{
	previous_output = current_output;
	current_output = cur_output;
}

float Smoothing::SmoothingOutput()
{
	float smoothing_output = ((current_output - previous_output) * ++output_count) / output_period + previous_output;

	if(output_count==output_period)
		output_count = 0;

	return smoothing_output;
}

}
