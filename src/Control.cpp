#include "Control.h"
#include "helper_functions.h"
#define PI 3.1415926f

Control::Control() : sensors(&controller) {
	controller.set_target_velocity(0, 0, 0);
	controller.set_pwm(controller.left_wheel, 0);
	controller.set_pwm(controller.right_wheel, 0);
}

void Control::start_threads() {
	controller.start_thread();
	sensors.ekf_thread_start();
	control_thread.start(callback(this, &Control::pose_control_thread));
}

void Control::set_target_pose(float x, float y, float theta) {
	swap = false;
	target = {x, y, theta};
}

void Control::pose_control_thread() {
	while (true) {
		auto target_vel = vector_control(target.theta, 0);
		auto target_wheel_vel = get_target_wheel_velocity(target_vel);
//		auto target_wheel_vel = get_target_wheel_velocity({0, 10});
		controller.set_target_velocity(target_wheel_vel);
		Thread::wait(10);
	}
}

WheelVelocity Control::get_target_wheel_velocity(TargetVelocity target) const {
	return {target.v - target.w * ROBOT_SIZE / 2,
			target.v + target.w * ROBOT_SIZE / 2};
}

PolarPose Control::get_polar_pose(TargetPose target) const {
	auto& pose = sensors.get_pose();
	float error = std::sqrt(std::pow(target.x - pose.x, 2.0f) + std::pow(target.y - pose.y, 2.0f));
	float robot_to_targ = std::atan2(target.y - pose.y, target.x - pose.x);
	float theta = wrap(robot_to_targ - target.theta);
	float alpha = wrap(robot_to_targ - pose.theta);
	return {error, -theta, -alpha};
}

bool correct_heading(PolarPose pose) {
	float theta_error = 0.1;
	return std::abs(pose.alpha) < theta_error;
}

TargetVelocity Control::control(TargetPose target) {
	PolarPose pose = get_polar_pose(target);

	if(pose.error < 0.01 || swap) {
//		swap = true;
		return vector_control(target.theta, velocity);
	}
	else return control_law(pose, 0.8);
}

TargetVelocity Control::vector_control(float target_theta, float velocity) const {
	return {velocity, 15 * wrap(target_theta - sensors.get_pose().theta)};
}

TargetVelocity Control::control_law(PolarPose pose, float vmax) const {
	float k = (-1 / pose.error) *
			(k2 * (pose.alpha - std::atan(-k1 * pose.theta))
			 + (1 + k1 / (1 + std::pow(k1 * pose.theta, 2.0f))) * std::sin(pose.alpha));
	float v = vmax/(1 + B * std::pow(k, 2.0f));
	float w = v * k;
	return {v, w};
}

