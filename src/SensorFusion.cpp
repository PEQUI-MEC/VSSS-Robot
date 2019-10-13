//
// Created by Thiago on 18/05/18.
//

#include <array>
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
	wait(5);
	calib();
	thread_ekf.start(callback(this, &SensorFusion::ekf_thread));
}

std::array<float, 2> model(const Controls & read, float gravity) {
//	auto theta_y = std::asin(read.acc(0) / gravity);
//	auto theta_x = std::acos(read.acc(2) / (std::cos(theta_y) * gravity));
	auto theta_x = atan2(read.acc(1), read.acc(2));
	auto theta_y = atan2(-read.acc(0), std::sqrt(read.acc(1)*read.acc(1) + read.acc(2)*read.acc(2)));
	return {theta_x, theta_y};
}

void SensorFusion::ekf_thread() {
	Timer timer_ekf;
	timer_ekf.start();
	timer_mag.start();
	Timer bench;
	bench.start();

	while (true) {
		if (timeout.read_ms() > 500) {
			stop_and_wait();
			timer_ekf.reset();
		}
		int time_us = timer_ekf.read_us();
		if (time_us > EKF_PERIOD_US || new_vision_data) {
			timer_ekf.reset();
			float time = time_us / 1E6f;

			auto controls = imu.read_gyro_acc();
			controls.gyro -= offsets.gyro;

			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;

			bench.reset();
			ekf.predict(controls, WheelVelocity{wheel_vel.vel_left, wheel_vel.vel_right}, time);

			if (new_vision_data) {
				ekf.update_on_vision_data(vision.to_vec());
			} else {
				opt_mag mag_data = read_magnetometer();

				SensorData sensor_data(mag_data.mag_theta,
									   controls.gyro(2),
									   wheel_vel.vel_left,
									   wheel_vel.vel_right);

				ekf.model.use_encoders(wheel_vel.new_data);
				ekf.model.use_magnetometer(mag_data.valid);

				ekf.update_on_sensor_data(sensor_data.to_vec());
			}

			btime = bench.read_us();
		}
	}
}

//float SensorFusion::acc_model(AccRealData &acc, float gyro_rate) {
//	float theta = get_pose().theta_y;
//	float w2 = std::pow(gyro_rate, 2.0f);
//	ax_raw = acc.x;
//	ay = acc.y - acc_offset_y;
//	ax = (acc.x + gravity * std::sin(theta))
//		 / std::cos(theta);
//
////	alpha = (w2 * r_cos - ay) / r_sin;
//	ar = ax - w2 * r_sin;
////	ar_alpha_fix = ax - w2 * r_sin - alpha * r_cos;
////	if (w2 != 0) {
////		r_sin = ax / w2;
////		r_cos = ay / w2;
////	}
//	return ar;
//}


void SensorFusion::calib() {
	Eigen::Vector3f acc, gyro;
	float gravity_acm = 0, theta_x_acm = 0, theta_y_acm = 0;
	acc.setZero();
	gyro.setZero();

	constexpr uint32_t sample_size = 500;
	for (uint32_t i = 0; i < sample_size; ++i) {
		auto read = imu.read_gyro_acc();
		acc += read.acc;
		gyro += read.gyro;
		auto read_gravity = read.acc.norm();
		gravity_acm += read_gravity;
//		theta_y_acm += std::atan2(-read.acc(2), -read.acc(0)) + PI/2;
		auto ac = model(read, read_gravity);
//		auto read_theta_y = std::asin(read.acc(0) / read_gravity);
//		theta_y_acm += read_theta_y;
//		theta_x_acm += std::acos(read.acc(2) / (std::cos(read_theta_y) * read_gravity));
		theta_x_acm += ac[0];
		theta_y_acm += ac[1];
		wait_ms(5);
	}

	offsets.gyro = gyro / sample_size;
	offsets.acc = acc / sample_size;
	gravity = gravity_acm / sample_size;
	theta_x = theta_x_acm / sample_size;
	theta_y = theta_y_acm / sample_size;
}

void SensorFusion::calibration() {
	float gyro_acc = 0;
	float gyro_acc_x = 0;
	float acc_ay = 0;
	float gravity_acc  = 0;
	float theta_acc = 0;
	constexpr uint32_t sample_size_gyro = 500;
	for (uint32_t i = 0; i < sample_size_gyro; ++i) {
		gyro_acc += imu.read_gyro();
		gyro_acc_x += imu.read_gyro_x();
		auto acc = imu.read_acc_real();
		gravity_acc += std::sqrt(std::pow(acc.x, 2.0f)
								 + std::pow(acc.z, 2.0f));
		theta_acc += std::atan2(-acc.z, -acc.x) + PI/2;
		acc_ay += acc.y;
		wait_ms(5);
	}
	gravity = gravity_acc / sample_size_gyro;
	ekf.x(5, 0) = theta_acc / sample_size_gyro;
	ekf.COV(5, 5) = 0.0001f;
	gyro_offset = gyro_acc / sample_size_gyro;
	gyro_offset_y = gyro_acc_x / sample_size_gyro;
	acc_offset_y = acc_ay / sample_size_gyro;
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
	ekf.COV.setIdentity();
	if(thread_ekf.get_state() != Thread::WaitingThreadFlag) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
	}
	timeout.reset();
}

void SensorFusion::resume_thread() {
	if(thread_ekf.get_state() == Thread::WaitingThreadFlag) {
		thread_ekf.signal_set(CONTINUE_SIGNAL);
	}
}

Pose SensorFusion::get_pose() const {
	return Pose(ekf.x);
}


