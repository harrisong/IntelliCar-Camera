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
	void TurnControl();
	void MoveMotor();

	void DrawCenterPixelAndPrintEquation();
	int GetRotationInstruction();
	int GetPixel(const Byte* src, const int x, const int y);

	void Run();

private:
	Car m_car;
	KF m_gyro_kf;
	float m_gyro;
	uint16 n;
	uint16 n2;
	int16_t m_speed1, m_speed2;
	int32 m_position, m_target_position;
	int32 m_encoder_speed2;
	bool m_dir1, m_dir2;
	libutil::PidController<int16_t, int16_t> m_balance_pid;
	libutil::PidController<int32, int32> m_speed_pid;

	int m_count;
};

}

#endif /* CAMERA_APP_H_ */
