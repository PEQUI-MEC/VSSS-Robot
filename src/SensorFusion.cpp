//
// Created by Thiago on 18/05/18.
//

#include "SensorFusion.h"

#define MOTOR_REVOLUTION_PER_WHEEL_REV 75.8126f
#define PULSES_PER_REVOLUTION 12
#define PI 3.141592f

SensorFusion::SensorFusion() :
		imu(IMU_SDA_PIN, IMU_SCL_PIN),
		left_encoder(ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2),
		right_encoder(ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2) {
	timer_ekf.start();
	timer_mag.start();
	offset_update_timer.start();
	encoder_timer.start();
}

void SensorFusion::update_gyro_bias(float measured_gyro_rate) {
//	Predict
	auto offset_pred = gyro_offset;
	auto cov_pred = gyro_offset_cov + 0.0001f * offset_update_timer.read_us()/1E6f;
	offset_update_timer.reset();

//	Update
	auto offset_measured = measured_gyro_rate;
	auto K = cov_pred / (cov_pred + 0.02857541f);
	gyro_offset = offset_pred + K * (offset_measured - offset_pred);
	gyro_offset_cov = (1 - K) * cov_pred;
}

void SensorFusion::measure_initial_gyro_bias() {
	float acm = 0;
	const int SAMPLE_SIZE_OFFSET = 200;
	for (int i = 0; i < SAMPLE_SIZE_OFFSET; i++) {
		acm += imu.read_gyro();
		Thread::wait(5);
	}
	gyro_offset = acm/SAMPLE_SIZE_OFFSET;
}

bool SensorFusion::is_stopped(float gyro_rate) {
	return std::abs(gyro_rate) < 0.01 &&
			left_encoder.get_velocity() == 0 &&
			right_encoder.get_velocity() == 0;
}

bool SensorFusion::update_encoder_estimation() {
	int time_us = encoder_timer.read_us();
	if (time_us > 10000) {
		encoder_timer.reset();
		left_encoder.update_wheel_velocity();
		right_encoder.update_wheel_velocity();
		ekf.pose.left_wheel_vel = left_encoder.get_velocity();
		ekf.pose.right_wheel_vel = right_encoder.get_velocity();
		return true;
	} else {
		ekf.pose.left_wheel_vel = left_encoder.get_velocity();
		ekf.pose.right_wheel_vel = right_encoder.get_velocity();
		return false;
	}
}

bool SensorFusion::update_estimation() {
	bool updated_encoder = update_encoder_estimation();

	int time_us = timer_ekf.read_us();
	float time = time_us/1E6f;

	float gyro_rate_with_offset = imu.read_gyro();
	float gyro_rate = gyro_rate_with_offset - gyro_offset;

	if (new_vision_data) {
		new_vision_data = false;
		ekf.predict(time,
			left_encoder.get_acceleration(), right_encoder.get_acceleration(),
			gyro_rate - prev_mesure.gyro_w);
		ekf.update_camera(vision);
	} else if (time_us > EKF_PERIOD_US) {
		//if (is_stopped(gyro_rate)) {
		//	update_gyro_bias(gyro_rate_with_offset);
		//}
		opt_mag mag_data = read_magnetometer();
		measurement_data data = {mag_data.mag_theta,
								gyro_rate,
								left_encoder.get_velocity(),
								right_encoder.get_velocity()};
		ekf.predict(time,
			left_encoder.get_acceleration(), right_encoder.get_acceleration(),
			gyro_rate - prev_mesure.gyro_w);
		ekf.update(data, mag_data.valid, true);
		prev_mesure = data;
		timer_ekf.reset();
	}

	prev_mesure.gyro_w = gyro_rate;
	return updated_encoder;
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
