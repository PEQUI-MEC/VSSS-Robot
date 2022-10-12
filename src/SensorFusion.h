//
// Created by Thiago on 18/05/18.
//

#ifndef VSSS_SENSORFUSION_H
#define VSSS_SENSORFUSION_H

#include <lib/QEI/QEI.h>
#include "IMU.h"
#include "EKF.h"
#include "Controller.h"

#define EKF_PERIOD_US 1000

struct opt_mag {
	bool valid;
	float mag_theta;
};

class SensorFusion {
	public:
		IMU imu;
		Controller* controller;

		EKF ekf;
		Timer timer_mag;

		vision_data vision{};
		bool new_vision_data = false;
		float mag_offset = 0;
		volatile bool wait = false;
		measurement_data prev_mesure{};

		void update_estimation();
		opt_mag read_magnetometer();

	public:
		bool no_vision = true;
		volatile bool reset_cov = false;

		float gyro_offset = 0;
		float gyro_offset_cov = 0.0001;
		float gyro_measured = 0;
		Timer offset_update_timer;

		explicit SensorFusion(Controller *controler_ptr);
		pose_data get_pose();
		void set_vision_data(float x, float y, float theta);
};

#endif //VSSS_SENSORFUSION_H
