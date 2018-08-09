#include "VFO.h"

VFO::VFO() : sensors(&controller) {
	controller.set_target_velocity(0, 0, 0);
	controller.set_pwm(controller.left_wheel, 0);
	controller.set_pwm(controller.right_wheel, 0);
}

void VFO::start_threads() {
	controller.start_thread();
	sensors.ekf_thread_start();
	control_thread.start(callback(this, &VFO::pose_control_thread));
}

void VFO::set_target_pose(float x, float y, float theta) {
	target = {{x, y}, theta};
}

void VFO::pose_control_thread() {
	while (true) {
		float error = std::sqrt(std::pow(target.position.x - sensors.get_pose().x, 2.0f)
								+ std::pow(target.position.y - sensors.get_pose().y, 2.0f));
		auto target_vel = [&]() {
			if(error > 0.01) return control_law(target);
			else return TargetVelocity{0, 20 * wrap(target.theta - sensors.get_pose().theta)};
		}();
		auto target_wheel_vel = get_target_wheel_velocity(target_vel);
		controller.set_target_velocity(target_wheel_vel);
		Thread::wait(10);
	}
}

WheelVelocity VFO::get_target_wheel_velocity(TargetVelocity target) const {
	return {target.v - target.w * ROBOT_SIZE / 2,
			target.v + target.w * ROBOT_SIZE / 2};
}

TargetVelocity VFO::control_law(TargetPose target) {
	H h = convergence_field(target);
	const Vector g2 = Vector(sensors.get_pose().theta);
	const float cos_alpha = (g2 * h.hs) / (g2.modulus() * h.hs.modulus());
	return {h.hs.modulus() * cos_alpha, h.h1};
}

H VFO::convergence_field(TargetPose target) {
	const Vector hs = kp * position_error(target.position) + ref_velocity(target);
	float h1 = k1 * orientation_error(hs) + ref_theta_dot(hs, target);
	return {h1, hs};
}

Vector VFO::position_convergence_field_dot(TargetPose target) {
	return -kp * position_error_dot() + ref_velocity_dot(target);
}

Vector VFO::ref_velocity(TargetPose target) {
	return -n * direction *
			position_error(target.position).modulus() *
			Vector(target.theta);
}

Vector VFO::ref_velocity_dot(TargetPose target) {
	const Vector error = position_error(target.position);
	return -n * direction *
			(error * position_error_dot()) / error.modulus() *
			Vector(target.theta); // g2t
}

Vector VFO::position_error(Point target) {
	return {target.x - sensors.get_pose().x,
			target.y - sensors.get_pose().y};
}

Vector VFO::position_error_dot() {
	return -sensors.get_pose().v * Vector(sensors.get_pose().theta);
}

float VFO::orientation_error(Vector hs) {
	return wrap(ref_theta(hs) - sensors.get_pose().theta);
}

float VFO::ref_theta(Vector hs) {
	return std::atan2(direction * hs.y, direction * hs.x);
}

float VFO::ref_theta_dot(Vector hs, TargetPose target) {
	const Vector hs_dot = position_convergence_field_dot(target);
	return (hs_dot.y * hs.x - hs_dot.x * hs.y) /
			(std::pow(hs.x, 2.0f) + std::pow(hs.y, 2.0f));
}
