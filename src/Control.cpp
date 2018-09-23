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
	timeout.start();
}

void Control::stop_and_sleep() {
	state = ControlState::None;
	controller.stop = true;
	sensors.stop = true;
	if(control_thread.get_state() != Thread::WaitingThreadFlag) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
	}
}

void Control::set_ekf_vision_data(float x, float y, float theta) {
	sensors.set_vision_data(x, y, theta);
	timeout.reset();
}

void Control::set_target_pose(float x, float y, float theta, bool stop_afterwards, float velocity) {
	target = {x, y, theta, velocity};
	this->stop_afterwards = stop_afterwards;
	state = ControlState::Pose;
	timeout.reset();
}

void Control::set_vector_control(float target_theta, float velocity) {
	target = {0, 0, target_theta, velocity};
	state = ControlState::Vector;
	timeout.reset();
}

void Control::set_target_orientation(float theta) {
	target = {0, 0, theta, 0};
	state = ControlState::Orientation;
	timeout.reset();
}

void Control::set_target_position(float x, float y, float velocity) {
	target = {x, y, 0, velocity};
	state = ControlState::Position;
	timeout.reset();
}

void Control::set_ang_vel_control(float angular_velocity) {
	target = {0, 0, 0, angular_velocity};
	state = ControlState::AngularVel;
	timeout.reset();
}

void Control::pose_control_thread() {
	while (true) {
		if (state == ControlState::None) stop_and_sleep();
		auto target = maybe_backwards(this->target);
		auto target_vel = [&]() {
//			if (timeout.read_ms() > 500) return TargetVelocity{0, 0};
			switch (state) {
				case ControlState::Pose:
					return pose_control(target);
				case ControlState::Position:
					return position_control(target);
				case ControlState::Vector:
					return vector_control(target.theta, target.velocity);
				case ControlState::Orientation:
					return orientation_control(target.theta);
				case ControlState::AngularVel:
					return TargetVelocity{0, target.velocity};
				case ControlState::None :
					stop_and_sleep();
					return TargetVelocity{0, 0};
				default:
					return TargetVelocity{0, 0};
			}
		}();
		auto target_wheel_vel = get_target_wheel_velocity(target_vel);
		controller.set_target_velocity(target_wheel_vel);
		Thread::wait(10);
	}
}

WheelVelocity Control::get_target_wheel_velocity(TargetVelocity target) const {
	return {target.v - target.w * ROBOT_SIZE / 2,
			target.v + target.w * ROBOT_SIZE / 2};
}

TargetVelocity Control::pose_control(Target target) {
	const PolarPose pose = get_polar_pose(target);
	if(pose.error < 0.02) {
		return vector_control(target.theta, target.velocity);
	} else {
		return control_law(pose, target.velocity);
	}
}

TargetVelocity Control::position_control(Target target) {
	auto pose = sensors.get_pose();
	float target_theta = std::atan2(target.y - pose.y,
									target.x - pose.x);
	float error = std::sqrt(std::pow(target.x - pose.x, 2.0f)
							+ std::pow(target.y - pose.y, 2.0f));
	if (error < 0.02) {
		state = ControlState::None;
		return {0, 0};
	} else return vector_control(target_theta, target.velocity * std::sqrt(error));
}

TargetVelocity Control::vector_control(float target_theta, float velocity) const {
	return {velocity, 15 * wrap(target_theta - sensors.get_pose().theta)};
}

TargetVelocity Control::orientation_control(float theta) {
	return {0, 15 * wrap(theta - sensors.get_pose().theta)};
}

PolarPose Control::get_polar_pose(Target target) const {
	auto pose = sensors.get_pose();
	float error = std::sqrt(std::pow(target.x - pose.x, 2.0f) + std::pow(target.y - pose.y, 2.0f));
	float robot_to_targ = std::atan2(target.y - pose.y, target.x - pose.x);
	float theta = wrap(robot_to_targ - target.theta);
	float alpha = wrap(robot_to_targ - pose.theta);
	return {error, -theta, -alpha};
}

TargetVelocity Control::control_law(PolarPose pose, float vmax) const {
	float k = (-1 / pose.error) *
			(k2 * (pose.alpha - std::atan(-k1 * pose.theta))
			 + (1 + k1 / (1 + std::pow(k1 * pose.theta, 2.0f))) * std::sin(pose.alpha));
	float v = vmax/(1 + B * std::pow(k, 2.0f));
	float w = v * k;
	return {v, w};
}

Target Control::maybe_backwards(Target target) {
	bool select = backwards_select(target.theta);
	if (select) return {target.x, target.y,
						wrap(target.theta + PI), -target.velocity};
	else return target;
}

bool Control::backwards_select(float target_theta) {
	auto theta = sensors.get_pose().theta;
	if(backwards_timer.read_ms() > 50) {
		bool backwards = std::abs(wrap(target_theta - theta)) > PI/2;
		if(previously_backwards != backwards) backwards_timer.reset();
		previously_backwards = backwards;
		return backwards;
	} else {
		return previously_backwards;
	}
}
