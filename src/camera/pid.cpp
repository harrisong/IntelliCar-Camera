/*
 * pid.cpp
 *
 *  Created on: 2014年6月19日
 *      Author: FaiFai
 */

#include "pid.h"
#include <libutil/misc.h>

namespace camera
{

PID::PID(const float setpoint, const float integral_limit, float _kp[], float _ki[], float _kd[], const int n, const int mode)
	: setpoint(setpoint), integral_limit(integral_limit), mode(mode), previous_error(0), current_error(0), total_error(0)
{
	kp = new float[n];
	ki = new float[n];
	kd = new float[n];
	for(int i=0; i<n; i++)
	{
		kp[i] = _kp[i];
		ki[i] = _ki[i];
		kd[i] = _kd[i];
	}
}

PID::~PID()
{
	delete[] kp;
	delete[] ki;
	delete[] kd;
}

void PID::UpdatePreviousError()
{
	previous_error = current_error;
}

void PID::UpdateCurrentError(const float cur_reading)
{
	current_error = setpoint - cur_reading;
	total_error += current_error;
}

void PID::ResetError(const float error)
{
	current_error = error;
	previous_error = error;
	total_error = error;
}

void PID::SetSetPoint(const float new_setpoint)
{
	setpoint = new_setpoint;
}

void PID::SetMode(const int new_mode)
{
	mode = new_mode;
}

void PID::SetKP(const float new_kp)
{
	kp[mode-1] = new_kp;
}

void PID::SetKI(const float new_ki)
{
	ki[mode-1] = new_ki;
}


void PID::SetKD(const float new_kd)
{
	kd[mode-1] = new_kd;
}

float PID::GetSetPoint() const
{
	return setpoint;
}

float PID::KP() const
{
	return kp[mode-1];
}

float PID::KI() const
{
	return ki[mode-1];
}

float PID::KD() const
{
	return kd[mode-1];
}

float PID::Proportional() const
{
	return kp[mode-1] * current_error;
}

float PID::Integral() const
{
	//if(integral_limit<0)
		return ki[mode-1] * total_error;

//	return ki[mode-1] * libutil::Clamp<float>(-integral_limit, total_error, integral_limit);
}

float PID::Derivative() const
{
	return kd[mode-1] * (current_error - previous_error);
}

}

