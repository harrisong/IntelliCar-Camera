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
	void SendImage();

	void Run();

private:
	Car m_car;
	KF gyro_kf;
	float _gyro;
	int _count;
	int16 Speed1, Speed2;
	int32 Position, TargetPosition;
	bool m_dir1, m_dir2;
	libutil::PidController<uint16, int16> m_balance_pid;
	libutil::PidController<uint16, int16> m_speed_pid;
};

}

#endif /* CAMERA_APP_H_ */
