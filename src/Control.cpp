#include "Control.h"
#include "helper_functions.h"
#define PI 3.1415926f

Control::Control() : sensors(&controller) {
	controller.set_target_velocity(0, 0, 0);
	controller.set_pwm(controller.left_wheel, 0);
	controller.set_pwm(controller.right_wheel, 0);
	backwards_timer.start();
}

void Control::start_threads() {
	controller.start_thread();
	sensors.ekf_thread_start();
	control_thread.start(callback(this, &Control::pose_control_thread));
}

void Control::resume_threads() {
	control_thread.signal_set(CONTINUE_SIGNAL);
	controller.continue_thread();
}

void Control::reset_timeout() {
	sensors.timeout.reset();
}

void Control::stop_and_sleep() {
	if(!sleep_enabled) return;

	state = ControlState::None;
	controller.stop = true;
	Thread::signal_wait(CONTINUE_SIGNAL);
	Thread::signal_clr(CONTINUE_SIGNAL);
	reset_timeout();
}

// Return from sleep
void Control::set_ekf_vision_data(float x, float y, float theta) {
	sensors.set_vision_data(x, y, theta);
}

void Control::set_target(ControlState control_type, Target target,
						 bool stop_afterwards) {
//	this->stop_afterwards = stop_afterwards;
	this->target = target;
	state = control_type;
	resume_threads();
}

TargetVelocity Control::set_stop_and_sleep() {
	state = ControlState::None;
	return {0, 0};
}

constexpr float MAX_VEL_CHANGE = 0.2;
float multiplier(float target, float vel) {
	if (std::abs(vel) < 0.01) {
		vel = (vel > 0) ? 0.01f : -0.01f;
	}
	if (std::abs(target) < 0.01) {
		target = (target > 0) ? 0.01f : -0.01f;
	}

	auto change = (target - vel)/vel;
	if (std::abs(change) < MAX_VEL_CHANGE) {
		return 1;
	} else {
		auto new_target = (MAX_VEL_CHANGE + 1) * vel;
		return new_target / target;
	}
}

TargetVelocity Control::limit_accel2(const TargetVelocity &target_vel) {
	auto wheel_vel = WheelVelocity{controller.left_wheel.velocity, controller.right_wheel.velocity};
	auto target_wheel_vel = get_target_wheel_velocity(target_vel);
	auto left_mult = multiplier(target_wheel_vel.left, wheel_vel.left);
	auto right_mult = multiplier(target_wheel_vel.right, wheel_vel.right);
	if (left_mult < right_mult) {
		return TargetVelocity(target_wheel_vel) * left_mult;
	} else {
		return TargetVelocity(target_wheel_vel) * right_mult;
	}
}

constexpr float MAX_VEL_DIFF = 0.15;
TargetVelocity Control::limit_accel(const TargetVelocity &target_vel) {
	auto wheel_vel = WheelVelocity{controller.left_wheel.velocity, controller.right_wheel.velocity};
	auto vel = TargetVelocity(wheel_vel);
	auto vel_diff = target_vel.v - vel.v;
	if (std::abs(vel_diff) > MAX_VEL_DIFF) {
		if (vel_diff > 0) {
			return TargetVelocity{vel.v + MAX_VEL_DIFF, target_vel.w};
		} else {
			return TargetVelocity{vel.v - MAX_VEL_DIFF, target_vel.w};
		}
	} else {
		return target_vel;
	}
}

void Control::pose_control_thread() {
	while (true) {
		if (state == ControlState::None ||
				sensors.timeout.read_ms() > 500) stop_and_sleep();

		auto pose = sensors.get_pose();

		auto target_vel = [&]() {
			switch (state) {
				case ControlState::Pose:
					return pose_control(sensors.get_pose(), this->target);
				case ControlState::Position:
					return position_control(pose, target);
				case ControlState::Vector:
					return vector_control(pose.theta, target.theta, target.velocity, true);
				case ControlState::Orientation:
					return orientation_control(pose, target.theta);
				case ControlState::AngularVel:
					return TargetVelocity{0, target.velocity};
				default:
					return TargetVelocity{0, 0};
			}
		}();

		auto target_wheel_vel = get_target_wheel_velocity(limit_accel(target_vel));
		controller.set_target_velocity(target_wheel_vel);
		Thread::wait(10);
	}
}

WheelVelocity Control::get_target_wheel_velocity(TargetVelocity target) const {
	return {target.v - target.w * ROBOT_SIZE / 2,
			target.v + target.w * ROBOT_SIZE / 2};
}

TargetVelocity Control::pose_control(Pose pose, Target target) {
	float state_to_targ = std::atan2(target.y - pose.y, target.x - pose.x);
	float state_to_ref = std::atan2(target.uvf_ref.y - pose.y, target.uvf_ref.x - pose.x);
	float fi = wrap(state_to_ref - state_to_targ);
	float uvf_target_theta = wrap(state_to_targ - uvf_n * fi);
	return vector_control(pose.theta,  uvf_target_theta, target.velocity, true);
}

constexpr float low_error_threshold = 0.3;
TargetVelocity Control::position_control(Pose pose, Target target) {
	float target_theta = std::atan2(target.y - pose.y,
									target.x - pose.x);
	float error = std::sqrt(std::pow(target.x - pose.x, 2.0f)
							+ std::pow(target.y - pose.y, 2.0f));
	float coefficient = std::log((target.velocity + 1) / low_error_threshold);
	float low_error_velocity = std::exp(coefficient * error) - 1;
	float velocity = (error > low_error_threshold) ? target.velocity : low_error_velocity;
	if (error < 0.01) return set_stop_and_sleep();
	else return vector_control(pose.theta, target_theta, velocity, true);
}

TargetVelocity Control::vector_control(float theta, float target_theta,
									   float velocity, bool enable_backwards) {
	auto error = wrap(target_theta - theta);
	if (enable_backwards && backwards_select(error)) {
		auto backwards_error = wrap(target_theta - (theta + PI));
		return {-velocity * std::cos(backwards_error), 7 * backwards_error};
	} else {
		return {velocity * std::cos(error), 7 * error};
	}
}

TargetVelocity Control::orientation_control(Pose pose, float theta) {
	return {0, 25 * wrap(theta - pose.theta)};
}

PolarPose Control::get_polar_pose(Pose pose, Target target) const {
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

bool Control::backwards_select(float theta_error) {
	if(backwards_timer.read_ms() > 50) {
		bool go_backwards = std::abs(theta_error) > PI/2;
		if(backwards != go_backwards) backwards_timer.reset();
		backwards = go_backwards;
		return go_backwards;
	} else {
		return backwards;
	}
}
