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
	void TurnControl();
	void MoveMotor();

	void DrawCenterPixelAndPrintEquation();
	int GetRotationInstruction();
	int GetPixel(const Byte* src, const int x, const int y);

	void Run();

private:
	Car m_car;
	float m_gyro;
	uint16 n;
	uint16 n2;
	int32_t m_balance_speed1, m_balance_speed2;
	int32_t m_control_speed1, m_control_speed2, m_total_speed1, m_total_speed2;
	int32_t m_real_current_speed, m_real_prev_speed, m_real_total_speed;
	int32_t prev_tempspeed, current_tempspeed;
	int32 m_position, m_target_position;
	int32 m_encoder_speed2;
	bool m_dir1, m_dir2;
	uint16_t current_time, prev_time, delta_time;
	libutil::PidController<int16_t, int32_t> m_balance_pid;
	libutil::PidController<int16_t, int32_t> m_speed_pid;
	libutil::PidController<int16_t, int32_t> m_position_pid;

	int16 encoder1,encoder2;
	int32_t m_count, m_speed_control_count;
};

}

#endif /* CAMERA_APP_H_ */
