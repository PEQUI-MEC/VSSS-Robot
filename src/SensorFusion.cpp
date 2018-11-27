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
		if (timeout.read_ms() > 500) {
			stop_and_wait();
			timer_ekf.reset();
		}
		int time_us = timer_ekf.read_us();
		if (time_us > EKF_PERIOD_US || new_vision_data) {
			timer_ekf.reset();
			float time = time_us / 1E6f;

			float gyro_rate = imu.read_gyro() - gyro_offset;
//			float acc = imu.read_acc_components().y - acc_offset;

//			auto acc = imu.read_acc();
//			auto controls = acc_model(acc.y - acc_offset_y,
//									  -(acc.x - acc_offset_x), gyro_rate);

//			x_acc = acc.x - acc_offset_x;
//			y_acc = acc.y - acc_offset_y;

//			last_controls.lin_accel = controls.lin_accel;
//			last_controls.ang_accel = controls.ang_accel;
//			Controls controls(acc.x - acc_offset_x, 0);

			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;

//			auto centripetal_x = std::pow(gyro_rate, 2.0f) * 0.02f * std::sin(0.785398f);
//			auto centripetal_y = std::pow(gyro_rate, 2.0f) * 0.02f * std::cos(0.785398f);

//			auto centripetal_x = std::pow(gyro_rate, 2.0f) * 0.01335404f;
//			auto centripetal_y = std::pow(gyro_rate, 2.0f) * 0.01615549f;

//			x_acc_fixed = x_acc + centripetal_x;
//			y_acc_fixed = y_acc - centripetal_y;

//			if (gyro_rate != 0) {
//				float w_sq = std::pow(gyro_rate, 2.0f);
//				A = y_acc / w_sq;
//				B = x_acc / w_sq;
//			}

			float acc = (wheel_vel.vel_left_accel +
					wheel_vel.vel_right_accel) / (2 * time);
			Controls controls(acc,
							  (gyro_rate - previous_w) / time);

			ukf.predict(controls.to_vec(), time);

			if (new_vision_data) {
				ukf.update_on_vision_data(vision.to_vec());
			} else {
				opt_mag mag_data = read_magnetometer();

				SensorData sensor_data(mag_data.mag_theta,
									   gyro_rate,
									   wheel_vel.vel_left,
									   wheel_vel.vel_right);

				ukf.model.use_encoders(wheel_vel.new_data);
				ukf.model.use_magnetometer(mag_data.valid);

				ukf.update_on_sensor_data(sensor_data.to_vec());
			}

			previous_w = gyro_rate;
		}
	}
}

void SensorFusion::gyro_calib() {
	float gyro_acc = 0;
	float acc_ax = 0;
	float acc_ay = 0;
	constexpr uint32_t sample_size_gyro = 500;
	for (uint32_t i = 0; i < sample_size_gyro; ++i) {
		gyro_acc += imu.read_gyro();
		auto acc = imu.read_acc();
		acc_ax += acc.x;
		acc_ay += acc.y;
		wait_ms(5);
	}
	gyro_offset = gyro_acc / sample_size_gyro;
	acc_offset_x = acc_ax / sample_size_gyro;
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
		mag_offset = vision.theta;
		no_vision = false;
	}
}

opt_mag SensorFusion::read_magnetometer() {
	return {false, 0};

//	bool use_mag = timer_mag.read_ms() > 10;
//	if (use_mag) {
//		timer_mag.reset();
		return {true, imu.read_mag() - mag_offset};
//	} else return {false, prev_mag};
}

void SensorFusion::stop_and_wait() {
	ukf.COV.setIdentity();
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
	return Pose(ukf.x);
}
