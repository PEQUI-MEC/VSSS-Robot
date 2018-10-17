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

	float mag_offset = 0;

	float previous_w = 0;
	int e_time = 0;

	void ekf_thread();
	opt_mag read_magnetometer();
	void gyro_calib();

	volatile Controls last_controls{0,0};
	volatile float x_acc = 0;
	volatile float x_acc_fixed = 0;
	volatile float y_acc = 0;
	volatile float y_acc_fixed = 0;

	public:
	bool no_vision = true;
	float gyro_offset = 0;
	volatile float acc_offset_x = 0;
	volatile float acc_offset_y = 0;

	volatile float A = 0;
	volatile float B = 0;

	explicit SensorFusion(Controller *controler_ptr);
	void ekf_thread_start(I2C *imu_i2c);
	Pose get_pose() const;
	void set_vision_data(float x, float y, float theta);
	void stop_and_wait();
	void resume_thread();
};

#endif //VSSS_SENSORFUSION_H
