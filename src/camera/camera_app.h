/*
 * camera_app.h
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef CAMERA_APP_H_
#define CAMERA_APP_H_
#include <log.h>
#include <libutil/misc.h>
//#include <libutil/pid_controller.h>
//#include <libutil/pid_controller.tcc>
#include "camera/car.h"
#include "camera/helper.h"
#include "camera/pid.h"
#include "camera/smoothing.h"


namespace camera
{

class CameraApp
{
public:
	CameraApp();
	~CameraApp();
	void BalanceControl();
	void PositionControl();
	void SpeedControl();
	void SpeedControlOutput();
	void ProcessImage(int n);
	void TurnControl();
	void TurnControlOutput();
	void MoveMotor();

	void Run();

	void AutoMode();
	void PidMode();
	void AccelAndGyroMode();
	void EncoderMode();
	void CameraMode();
	void ParadeMode();
	void BalanceOnlyMode();
	void CameraMoveMode();
	void BalanceAndSpeedMode();
	void MoveMotorMode();
	void UartMode();
	void CalGyroMode();
	void SpeedModeOne();
	void SpeedModeTwo();
	void TimeMeasurementMode();
	void SpeedToMotorMode();

private:
	Car m_car;
	Helper m_helper;

	int32_t m_balance_speed[2];
	int32_t m_control_speed[2];
	int32_t m_turn_speed[2];
	int32_t m_total_speed[2];
	bool m_dir[2];
	int white_dot[2];

	PID m_speed_pid;
	PID m_turn_pid;
	PID m_balance_pid;

	Smoothing speed_smoothing;
	Smoothing turn_smoothing;

	float m_gyro;

	int32_t m_encoder_2;
	int32_t m_count;

	int num_finished_row;

	const Byte* src;

};

}

#endif /* CAMERA_APP_H_ */
