/*
 * camera_app.h
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef CAMERA_APP_H_
#define CAMERA_APP_H_
#include <libutil/misc.h>
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
	void TurnControl();
	void SendToMotor();
	void SendImage();

	void Run();

private:
	Car m_car;
	KF gyro_kf;
	float _gyro;
	int _count;
	int16 Speed1, Speed2;
	int32 Position, TargetPosition;
};

}

#endif /* CAMERA_APP_H_ */
