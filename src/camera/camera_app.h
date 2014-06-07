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
	void TurnControlOutput();
	void MoveMotor();

	void PrintCam();

	void EdgeDetection(const Byte* src, const int y);
	int GetPixel(const Byte* src, const int x, const int y);
	int GetCenterPoint(const Byte* src);
	int GetEdge(const Byte* src, const int center, const int direction);

	void Printline(uint8_t y, const char* s);
	void Printline(uint8_t y, const char* s, bool inverted);
	void PrintPtr(uint8_t y);

	void Run();

private:
	Car m_car;
	float m_gyro;

	int32_t m_balance_speed1, m_balance_speed2;
	int32_t m_control_speed1, m_control_speed2;
	int32_t m_total_speed1, m_total_speed2;
	int32_t m_encoder_2;
	int32_t prev_tempspeed, current_tempspeed;

	int32_t m_turn_speed1, m_turn_speed2;

	bool m_dir1, m_dir2;
	libutil::PidController<int16_t, int32_t> m_balance_pid;
	libutil::PidController<int16_t, int32_t> m_speed_pid;

	libsc::Lcd m_lcd;

	int32_t m_count;
};

}

#endif /* CAMERA_APP_H_ */
