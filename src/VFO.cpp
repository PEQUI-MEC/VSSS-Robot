#include "VFO.h"

//void VFO::pose_control_thread() {
//	while (true) {
//		float error = std::sqrt(std::pow(target.position.x - sensors.get_pose().x, 2.0f)
//								+ std::pow(target.position.y - sensors.get_pose().y, 2.0f));
//		auto target_vel = [&]() {
//			if(error > 0.01) return control_law(target);
//			else return TargetVelocity{0, 20 * wrap(target.theta - sensors.get_pose().theta)};
//		}();
//		auto target_wheel_vel = get_target_wheel_velocity(target_vel);
//		controller.set_target_velocity(target_wheel_vel);
//		Thread::wait(10);
//	}
//}

TargetVelocity VFO::control_law(Target target, const Pose &pose) {
	H h = convergence_field(target, pose);
	const Vector g2 = Vector(pose.theta);
	const float cos_alpha = (g2 * h.hs) / (g2.modulus() * h.hs.modulus());
	return {h.hs.modulus() * cos_alpha, h.h1};
}

H VFO::convergence_field(Target target, Pose pose) {
	const Vector hs = kp * position_error(pose.position, target.position) + ref_velocity(target, pose.position);
	float h1 = k1 * orientation_error(hs, pose) + ref_theta_dot(hs, target, pose);
	return {h1, hs};
}

Vector VFO::position_convergence_field_dot(Target target, Pose pose) {
	return -kp * position_error_dot(pose) + ref_velocity_dot(target, pose);
}

Vector VFO::ref_velocity(Target target, Point position) {
	return -n * direction *
		   position_error(position, target.position).modulus() *
		   Vector(target.theta);
}

Vector VFO::ref_velocity_dot(Target target, Pose pose) {
	const Vector error = position_error(pose.position, target.position);
	return -n * direction *
		   (error * position_error_dot(pose) / error.modulus() *
		   Vector(target.theta)); // g2t
}

Vector VFO::position_error(Point position, Point target) {
	return {target.x - position.x,
			target.y - position.y};
}

Vector VFO::position_error_dot(Pose pose) {
	return -pose.v * Vector(pose.theta);
}

float VFO::orientation_error(Vector hs, Pose pose) {
	return wrap(ref_theta(hs) - pose.theta);
}

float VFO::ref_theta(Vector hs) {
	return std::atan2(direction * hs.y, direction * hs.x);
}

float VFO::ref_theta_dot(Vector hs, Target target, Pose pose) {
	const Vector hs_dot = position_convergence_field_dot(target, pose);
	return (hs_dot.y * hs.x - hs_dot.x * hs.y) /
			(std::pow(hs.x, 2.0f) + std::pow(hs.y, 2.0f));
}
