//
// Created by Thiago on 18/05/18.
//

#include "SensorFusion.h"
#include "Controller.h"
#include "PIN_MAP.h"

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

	while (true) {
		int time_us = timer_ekf.read_us();
		if (time_us > EKF_PERIOD_US || new_vision_data) {
			timer_ekf.reset();
			float time = time_us / 1E6f;

			float gyro_rate = imu.read_gyro() - gyro_offset;
			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;

			Controls controls((wheel_vel.vel_left_accel + wheel_vel.vel_right_accel) / 2,
							  gyro_rate - previous_w);

			ekf.predict(controls.to_vec(), time);

			if (new_vision_data) {
				ekf.update_on_vision_data(vision.to_vec());
			} else {
				opt_mag mag_data = read_magnetometer();

				SensorData sensor_data(mag_data.mag_theta,
									   gyro_rate,
									   wheel_vel.vel_left,
									   wheel_vel.vel_right);

				ekf.model.use_encoders(wheel_vel.new_data);
				ekf.model.use_magnetometer(mag_data.valid);

				ekf.update_on_sensor_data(sensor_data.to_vec());
			}

			previous_w = gyro_rate;
			if(stop) stop_and_wait();
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
	gyro_offset = acc / sample_size_gyro;
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
	if (no_vision) {
		mag_offset = vision.theta;
		no_vision = false;
	}
}

opt_mag SensorFusion::read_magnetometer() {
	return {false, 0};

//	bool use_mag = timer_mag.read_ms() > 10;
//	if (use_mag) {
//		timer_mag.reset();
//		return {true, imu.read_mag() - mag_offset};
//	} else return {false, prev_mesure.mag_theta};
}

void SensorFusion::stop_and_wait() {
	stop = false;
	ekf.COV.setIdentity();
	if(thread_ekf.get_state() != Thread::WaitingThreadFlag) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
	}
}

Pose SensorFusion::get_pose() const {
	return Pose(ekf.x);
}
