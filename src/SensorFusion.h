//
// Created by Thiago on 18/05/18.
//

#ifndef VSSS_SENSORFUSION_H
#define VSSS_SENSORFUSION_H

#include <lib/QEI/QEI.h>
#include "IMU.h"
#include "EKF.h"
#include "Wheel.h"

#define EKF_PERIOD_US 1000

struct opt_mag {
	bool valid;
	float mag_theta;
};

class SensorFusion {
	public:
		IMU imu;
		WheelEncoder left_encoder;
		WheelEncoder right_encoder;

		EKF ekf;
		Timer timer_mag;
		Timer timer_ekf;
		Timer encoder_timer;

		vision_data vision{};
		bool new_vision_data = false;
		float mag_offset = 0;
		volatile bool wait = false;
		measurement_data prev_mesure{};

		bool update_estimation();
		opt_mag read_magnetometer();

	public:
		bool no_vision = true;

		float gyro_offset = 0;
		float gyro_offset_cov = 0.0001;
		float gyro_measured = 0;
		Timer offset_update_timer;

		SensorFusion();
		pose_data get_pose();
		void set_vision_data(float x, float y, float theta);
		void reset_local_sensors();
		void update_gyro_bias(float measured_gyro_rate);
		bool is_stopped(float gyro_rate);
		void measure_initial_gyro_bias();
		bool update_encoder_estimation();
};

#endif //VSSS_SENSORFUSION_H
