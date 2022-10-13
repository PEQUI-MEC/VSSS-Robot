//
// Created by Thiago on 18/05/18.
//

#include "SensorFusion.h"

#define MOTOR_REVOLUTION_PER_WHEEL_REV 75.8126f
#define PULSES_PER_REVOLUTION 12
#define PI 3.141592f

SensorFusion::SensorFusion() :
		left_encoder(ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2),
		right_encoder(ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2) {
	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
}

void SensorFusion::update_estimation() {
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

			float gyro_rate = imu.read_gyro() - gyro_offset;
			
			ekf.predict(time, left_encoder.get_acceleration(), right_encoder.get_acceleration(), gyro_rate - prev_mesure.gyro_w);
			prev_mesure.gyro_w = gyro_rate;
			ekf.update_camera(vision);

		} else if(time_us > EKF_PERIOD_US && !wait) {
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds
			opt_mag mag_data = read_magnetometer();
			float gyro_rate = imu.read_gyro();
			gyro_measured = gyro_rate;

			if(std::abs(gyro_rate - gyro_offset) < 0.01 &&
					left_encoder.get_velocity() == 0 &&
					right_encoder.get_velocity() == 0) {
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
									 left_encoder.get_velocity(),
									 right_encoder.get_velocity()};

			ekf.predict(time, left_encoder.get_acceleration(), right_encoder.get_acceleration(), data.gyro_w - prev_mesure.gyro_w);
			prev_mesure = data;
			ekf.update(data, mag_data.valid, true);
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

void SensorFusion::reset_local_sensors() {
	left_encoder.reset();
	right_encoder.reset();
	ekf.COV(0,0) = 2;
	ekf.COV(1,1) = 2;
	ekf.COV(2,2) = 2;
}
