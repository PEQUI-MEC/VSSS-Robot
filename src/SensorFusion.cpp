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

			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, 0);
			ekf.update_camera(vision);

		} else if(time_us > EKF_PERIOD_US && !wait) {
			timer_ekf.reset();
			float time = time_us/1E6f; // Time in seconds

			auto wheel_vel = controller->encoder_vel;
			controller->encoder_vel.new_data = false;
			measurement_data data = {0,
									 0,
									 wheel_vel.vel_left,
									 wheel_vel.vel_right};

			ekf.predict(time, wheel_vel.vel_left_accel, wheel_vel.vel_right_accel, 0);
			prev_mesure = data;
			ekf.update(data, false, wheel_vel.new_data);
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

pose_data SensorFusion::get_pose() {
	return ekf.pose;
}
