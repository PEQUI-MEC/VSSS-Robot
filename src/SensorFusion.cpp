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
	wait(5);
	calibration();
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

			gyro_rate = imu.read_gyro() - gyro_offset;
			float gyro_rate_y_raw = (imu.read_gyro_x() - gyro_offset_y);
			float gyro_rate_y = gyro_rate_y_raw - gyro_yx * gyro_rate;
//			gyro_yx_m = gyro_rate_y_raw / gyro_rate;
			gyro_rate_y_m = gyro_rate_y;
//			float acc = imu.read_acc_components().y - acc_offset;

			auto acc = imu.read_acc_real();
			acc_real = acc;

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

//			float acc = (wheel_vel.vel_left_accel +
//					wheel_vel.vel_right_accel) / (2 * time);
			Controls controls(acc_model(acc, gyro_rate),
							  (gyro_rate - previous_w) / time,
							  gyro_rate_y);

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
		}
	}
}

float SensorFusion::acc_model(AccRealData &acc, float gyro_rate) {
	float theta = get_pose().theta_y;
	float w2 = std::pow(gyro_rate, 2.0f);
	ax_raw = acc.x;
	ay = acc.y - acc_offset_y;
	ax = (acc.x + gravity * std::sin(theta))
		 / std::cos(theta);

	alpha = (w2 * r_cos - ay) / r_sin;
	ar = ax - w2 * r_sin;
	ar_alpha_fix = ax - w2 * r_sin - alpha * r_cos;
//	if (w2 != 0) {
//		r_sin = ax / w2;
//		r_cos = ay / w2;
//	}
	return ar;
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
