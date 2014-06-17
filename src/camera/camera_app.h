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
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include "camera/car.h"
#include "camera/helper.h"


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

	float m_gyro;

	int32_t m_balance_speed1, m_balance_speed2;
	int32_t m_control_speed1, m_control_speed2;
	int32_t m_total_speed1, m_total_speed2;
	int32_t m_encoder_2;
	int32_t prev_tempspeed, current_tempspeed;
	int32_t prev_turn, current_turn;

	int32_t m_turn_speed1, m_turn_speed2;

	bool m_dir1, m_dir2;
	libutil::PidController<int16_t, int32_t> m_balance_pid;
	libutil::PidController<int16_t, int32_t> m_speed_pid;

	int32_t m_count;

	int LeftWhiteDot;
	int RightWhiteDot;

	const Byte* src;

};

}

#endif /* CAMERA_APP_H_ */
