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

	VisionData vision;
	bool new_vision_data = false;

	float mag_offset = 0;
	volatile bool stop = false;

	float previous_w = 0;
	int e_time = 0;

	void ekf_thread();
	opt_mag read_magnetometer();
	void gyro_calib();

	public:
	bool no_vision = true;
	float gyro_offset = 0;

	explicit SensorFusion(Controller *controler_ptr);
	void ekf_thread_start();
	Pose get_pose() const;
	void set_vision_data(float x, float y, float theta);
	void stop_and_wait();
};

#endif //VSSS_SENSORFUSION_H
