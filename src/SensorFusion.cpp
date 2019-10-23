//
// Created by Thiago on 18/05/18.
//

#include "SensorFusion.h"
#include "Controller.h"

#define MOTOR_REVOLUTION_PER_WHEEL_REV 75.8126f
#define PULSES_PER_REVOLUTION 12
#define PI 3.141592f

SensorFusion::SensorFusion(Controller *controler_ptr) {
	controller = controler_ptr;
	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
}

void SensorFusion::ekf_thread_start() {
	thread_ekf.start(callback(this, &SensorFusion::ekf_thread));
}

void SensorFusion::ekf_thread() {
	Timer timer_ekf;
	timer_ekf.start();
	timer_mag.start();
	offset_update_timer.start();

	while(true) {
		int time_us = timer_ekf.read_us();
		if(new_vision_data && !wait) {
			new_vision_data = false;
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds

			auto wheel_vel = controller->encoder_vel;
			float gyro_rate = imu.read_gyro() - gyro_offset;

			if (reset_cov) {
				ekf.COV(0,0) = 2;
				ekf.COV(1,1) = 2;
				ekf.COV(2,2) = 2;
				reset_cov = false;
			}
			
			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, gyro_rate - prev_mesure.gyro_w);
			prev_mesure.gyro_w = gyro_rate;
			ekf.update_camera(vision);

		} else if(time_us > EKF_PERIOD_US && !wait) {
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds
			opt_mag mag_data = read_magnetometer();
			float gyro_rate = imu.read_gyro();
			gyro_measured = gyro_rate;

			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;

			if(std::abs(gyro_rate - gyro_offset) < 0.01 && wheel_vel.vel_left == 0 && wheel_vel.vel_right == 0) {
//				Predict
				auto offset_pred = gyro_offset;
				auto cov_pred = gyro_offset_cov + 0.00001f * offset_update_timer.read_us()/1E6f;
				offset_update_timer.reset();

//				Update
				auto offset_measured = gyro_rate;
				auto K = cov_pred / (cov_pred + 0.02857541f);
				gyro_offset = offset_pred + K * (offset_measured - offset_pred);
				gyro_offset_cov = (1 - K) * cov_pred;
			}

			measurement_data data = {mag_data.mag_theta,
									 gyro_rate - gyro_offset,
									 wheel_vel.vel_left,
									 wheel_vel.vel_right};

			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, data.gyro_w - prev_mesure.gyro_w);
			prev_mesure = data;
			ekf.update(data, mag_data.valid, wheel_vel.new_data);
		}
	}
}

void SensorFusion::set_vision_data(float x, float y, float theta) {
	vision = {x/100, y/100, theta*PI/180};
	new_vision_data = true;
	if(no_vision) {
		mag_offset = vision.theta;
		no_vision = false;
	}
}

opt_mag SensorFusion::read_magnetometer() {
	return {false, 0};

	bool use_mag = timer_mag.read_ms() > 10;
	if(use_mag) {
		timer_mag.reset();
		return {true, imu.read_mag() - mag_offset};
	} else return {false, prev_mesure.mag_theta};
}

pose_data SensorFusion::get_pose() {
	return ekf.pose;
}
