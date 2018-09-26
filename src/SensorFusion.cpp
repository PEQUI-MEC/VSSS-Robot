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
}

void SensorFusion::ekf_thread_start() {
	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
	gyro_calib();
	thread_ekf.start(callback(this, &SensorFusion::ekf_thread));
}

void SensorFusion::ekf_thread() {
	Timer timer_ekf;
	timer_ekf.start();
	timer_mag.start();

	while(true) {
		int time_us = timer_ekf.read_us();
		if(new_vision_data && !wait) {
			new_vision_data = false;
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds

			auto wheel_vel = controller->encoder_vel;
			float gyro_rate = imu.read_gyro() - gyro_offset;
			
			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, gyro_rate - prev_mesure.gyro_w);
			prev_mesure.gyro_w = gyro_rate;
			ekf.update_camera(vision);

		} else if(time_us > EKF_PERIOD_US && !wait) {
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds

			opt_mag mag_data = read_magnetometer();
			float gyro_rate = imu.read_gyro() - gyro_offset;
			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;
			measurement_data data = {mag_data.mag_theta,
									 gyro_rate,
									 wheel_vel.vel_left,
									 wheel_vel.vel_right};

			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, data.gyro_w - prev_mesure.gyro_w);
//			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, 0);
			prev_mesure = data;
			ekf.update(data, mag_data.valid, wheel_vel.new_data);
		}
	}
}

void SensorFusion::gyro_calib() {
	float acc = 0;
	constexpr uint32_t sample_size_gyro = 500;
	for (uint32_t i = 0; i < sample_size_gyro; ++i) {
		acc += imu.read_gyro();
		wait_ms(5);
	}
	gyro_offset = acc/sample_size_gyro;
}

//void mag_calibration() {
//	IMU imu{};
//	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
//	Thread::wait(1000);
//
//	#define sample_size 5000
//	for (int i = 0; i < sample_size; ++i) {
////		robot->start_velocity_control(-0.05f, 0.05f);
//		auto data = imu.read_mag_components();
//		std::string msg = std::to_string(data.x) + ',' + std::to_string(data.y);
//		messenger.send_msg(msg);
//		Thread::wait(10);
//	}
//}

void SensorFusion::set_vision_data(float x, float y, float theta) {
	vision = {x, y, theta};
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

const pose_data & SensorFusion::get_pose() const {
	return ekf.pose;
}
