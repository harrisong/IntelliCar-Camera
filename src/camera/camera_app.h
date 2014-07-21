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
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>
#include "camera/car.h"
#include "camera/helper.h"
#include "camera/pid.h"
#include "camera/smoothing.h"
#include "math_tools.h"


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
	void ProcessImage();
	void TurnControl();
	void TurnControlOutput();
	void MoveMotor();
	bool isDestination(int startx, int y);

	void eStop();

	void Run();

	void AutoMode(int mode = -1);
	void AccelAndGyroMode();
	void EncoderMode();
	void CameraMode();
	void BalanceOnlyMode();
	void BalanceAndSpeedMode();
	static __ISR void Pit3Handler();
	static __ISR void ShiftAccel();

private:
	static void HardFaultHandler();
	static void PitIndicator();

	MOV_STRUCT acc_moving;
	float buffer[200];

	Car m_car;
	Helper m_helper;

	int32_t m_balance_speed[2];
	int32_t m_control_speed[2];
	int32_t m_turn_speed[2];
	int32_t m_total_speed[2];
	bool m_dir[2];
	int32_t white_dot[2];

	PID m_speed_pid;
	PID m_turn_pid;
	PID m_balance_pid;
	PID m_balance_derivative_pid;

	Smoothing speed_smoothing;
	Smoothing turn_smoothing;
	Smoothing speed_input_smoothing;

	float m_gyro;

	int32_t m_encoder_2;
	int32_t encoder_total;
	int32_t m_count;

	int32_t num_finished_row;

	const Byte* src;

	const libutil::TunableInt *tunableints[19];

	int e_stop;

	static CameraApp *m_instance;

	int start_row;
	int end_row;

	uint32_t t;

	bool stopped;

	int num_finished_laps;

	int32_t turn_error[2];

	int mode_chosen;
};

}

#endif /* CAMERA_APP_H_ */
