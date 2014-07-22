/*
 * pid.h
 *
 *  Created on: 2014年6月19日
 *      Author: FaiFai
 */

#ifndef PID_H_
#define PID_H_

namespace camera
{

class PID
{
	public:
		PID(float _setpoint[], const float integral_limit, float _kp[], float _ki[], float _kd[], const int n, const int mode=1);
		~PID();

		void UpdatePreviousError();
		void UpdateCurrentError(const float cur_error);
		void ResetError(const float error = 0.0);

		void SetIntegralLimit(const float limit);
		void SetMode(const int new_mode);
		void SetSetPoint(const float new_setpoint);
		void SetKP(const float new_kp);
		void SetKI(const float new_ki);
		void SetKD(const float new_kd);

		float GetSetPoint() const;

		float KP() const;
		float KI() const;
		float KD() const;
		float Proportional() const;
		float Integral() const;
		float Derivative() const;

	private:
		float* setpoint;
		float integral_limit;
		float* kp;
		float* ki;
		float* kd;
		float current_error;
		float previous_error;
		float total_error;
		int mode;
};

}



#endif /* PID_H_ */
