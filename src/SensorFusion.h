//
// Created by Thiago on 18/05/18.
//

#ifndef VSSS_SENSORFUSION_H
#define VSSS_SENSORFUSION_H

#include <lib/QEI/QEI.h>
#include "IMU.h"
#include "Controller.h"
#include "EkfModel.h"
#include "EKF2.h"

#define EKF_PERIOD_US 1000

struct opt_mag {
	bool valid;
	float mag_theta;
};

class SensorFusion {
	public:
	IMU imu;
	Controller *controller;

	EKF2 ekf;

	Thread thread_ekf;
	Timer timer_mag;
	Timer timeout;

	VisionData vision;
	bool new_vision_data = false;

	float previous_w = 0;

	void ekf_thread();
	opt_mag read_magnetometer();
	void calibration();
	float acc_model(AccRealData &acc, float gyro_rate);

//	volatile float x_acc = 0;
//	volatile float x_acc_fixed = 0;
//	volatile float y_acc = 0;
//	volatile float y_acc_fixed = 0;
	AccRealData acc_real{};

	public:
	bool no_vision = true;
	float gyro_offset = 0;
	float gyro_offset_y = 0;
	volatile float acc_offset_x = 0;
	volatile float acc_offset_y = 0;

//	float gyro_yx = 0.049127f;
	const float gyro_yx = 0.052512;
	float gyro_yx_m = 0;
	const float r_sin = 0.028734f;
	const float r_cos = 0.010513;
	float ax = 0;
	float ax_raw = 0;
	float ay = 0;
	float ar = 0;
	float ar_alpha_fix = 0;
	float alpha = 0;
	float rsin = -0.028737f;
	float rcos = 0.008833f;
	float gravity = 0;

	volatile float A = 0;
	volatile float B = 0;

	explicit SensorFusion(Controller *controler_ptr);
	void ekf_thread_start();
	Pose get_pose() const;
	void set_vision_data(float x, float y, float theta);
	void stop_and_wait();
	void resume_thread();
	float gyro_rate_y_m = 0;
	float gyro_rate = 0;
	int btime = 0;
};

#endif //VSSS_SENSORFUSION_H
