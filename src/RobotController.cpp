#include <cmath>
#include "RobotController.h"
#include "PIN_MAP.h"

#define PI 3.1415926f
#define ROBOT_LOOP_MS 10

RobotController::RobotController() {
	target.command = NO_CONTROL;
	backwards_timer.start();
}

void RobotController::control_loop() {
	if(msg_timeout_timer.read_ms() > msg_timeout_limit) {
		sensors->reset_cov = true;
	}
	if(msg_timeout_timer.read_ms() > msg_timeout_limit
		&& target.command != ORIENTATION_CONTROL) stop_and_wait();

	switch (target.command) {
		case VECTOR_CONTROL:
			vector_control();
			break;
		case POSITION_CONTROL:
			position_control();
			break;
		case ORIENTATION_CONTROL:
			orientation_control();
			break;
		case UVF_CONTROL:
			uvf_control();
			break;
		case NO_CONTROL:
//		Robot não faz nada, controle de velocidade é feito em Controller
			break;
		case SENSOR_CALIBRATION:
			sensor_calibration();
			break;
		default:
			break;
	}

	controller.control_loop();
}

void RobotController::uvf_control() {
	auto pose = sensors->get_pose();

	if(vel_acelerada < 0.3) vel_acelerada = 0.3;
	if(target.velocity == 0) {
		stop_and_wait();
		return;
	}

//	Computes uni-vector field target theta
	float state_to_targ = std::atan2(target.y - pose.y, target.x - pose.x);
	float state_to_ref = std::atan2(target.ref_y - pose.y, target.ref_x - pose.x);
	float fi = round_angle(state_to_ref - state_to_targ);
	float uvf_target_theta = round_angle(state_to_targ - uvf_n * fi);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(uvf_target_theta, pose.theta);
	if(move_backwards) uvf_target_theta = round_angle(uvf_target_theta + PI);

	float theta_error = round_angle(uvf_target_theta - pose.theta);

//	Decreases velocity for big errors
	if (std::abs(theta_error) > max_theta_error) {
		vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
	} else {
//		Applies acceleration until robot reaches target velocity
		if (vel_acelerada < target.velocity) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else {
			vel_acelerada = target.velocity;
		}
	}

	set_wheel_velocity_nonlinear_controller(theta_error, vel_acelerada, move_backwards);
}

void RobotController::vector_control() {
	if(vel_acelerada < 0.3) vel_acelerada = 0.3;
	if(target.velocity == 0) {
		stop_and_wait();
		return;
	}
	auto pose = sensors->get_pose();

//	Computes target_theta in direction of {target.x, target.y} before each control loop
	float target_theta = std::atan2(target.y - pose.y, target.x - pose.x);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(target_theta, pose.theta);
	if(move_backwards) target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Decreases velocity for big errors
	if (std::abs(theta_error) > max_theta_error) {
		vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
	} else {
//		Applies acceleration until robot reaches target velocity
		if (vel_acelerada < target.velocity) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else {
			vel_acelerada = target.velocity;
		}
	}

	set_wheel_velocity_nonlinear_controller(theta_error, vel_acelerada, move_backwards);
}

void RobotController::position_control() {
	auto pose = sensors->get_pose();
//	Stops after arriving at destination
	float position_error = std::sqrt(std::pow(pose.x - target.x, 2.0f) + std::pow(pose.y - target.y, 2.0f));
	if(target.velocity == 0 || position_error < 0.01) {
		stop_and_wait();
		return;
	}

	if(vel_acelerada < 0.3) vel_acelerada = 0.3;

//	Computes target_theta in direction of {target.x, target.y} before each control loop
	float target_theta = std::atan2(target.y - pose.y, target.x - pose.x);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(target_theta, pose.theta);
	if(move_backwards) target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Decreases velocity for big errors and limits maximum velocity
	if (std::abs(theta_error) > max_theta_error) {
		if(vel_acelerada > 0.8) {
			vel_acelerada = 0.8;
		} else if (vel_acelerada > 0.3) {
			vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
		}
	} else {
//		Applies acceleration until robot reaches target velocity
		float velocity_difference = target.velocity - vel_acelerada;
		if (velocity_difference > 0.2) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else if(velocity_difference < 0) {
			vel_acelerada = target.velocity;
		}
	}

	set_wheel_velocity_nonlinear_controller(theta_error, vel_acelerada, move_backwards);
}


void RobotController::orientation_control() {
	auto pose = sensors->get_pose();
	float target_theta = target.theta;

//	Activates backwards movement if theta_error > PI/2
	if(backwards_select(target_theta, pose.theta))
		target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Wheel velocities are always between 1 and -1
	float right_wheel_velocity = saturate(orientation_Kp * theta_error, 1);
	float left_wheel_velocity = saturate(-orientation_Kp * theta_error, 1);

	controller.set_target_velocity(left_wheel_velocity, right_wheel_velocity, target.velocity);
}

void RobotController::set_wheel_velocity_nonlinear_controller(float theta_error, float velocity, bool backwards) {
	float m = 1;
	if(backwards) m = -1;

	float right_wheel_velocity = m + std::sin(theta_error) + m*kgz*std::tan(m*theta_error/2);
	right_wheel_velocity = saturate(right_wheel_velocity,1);

	float left_wheel_velocity = m - std::sin(theta_error) + m*kgz*std::tan(-m*theta_error/2);
	left_wheel_velocity = saturate(left_wheel_velocity,1);

	controller.set_target_velocity(left_wheel_velocity, right_wheel_velocity, velocity);
}

void RobotController::start_uvf_control(float x, float y, float x_ref, float y_ref, float n, float velocity) {
	target.x = x;
	target.y = y;
	target.ref_x = x_ref;
	target.ref_y = y_ref;
	uvf_n = n;
	target.velocity = velocity;
	target.command = UVF_CONTROL;
	reset_timers();
}

void RobotController::start_vector_control(float theta, float velocity) {
//	target.theta is computed before each vector control loop, in direction of {target.x, target.y}
	target.x = 10*std::cos(theta * PI/180);
	target.y = 10*std::sin(theta * PI/180);
	target.velocity = velocity;
	target.command = VECTOR_CONTROL;
	reset_timers();
}

void RobotController::start_position_control(float x, float y, float velocity) {
	target.x = x;
	target.y = y;
	target.velocity = velocity;
	target.command = POSITION_CONTROL;
	reset_timers();
}

void RobotController::start_orientation_control(float theta, float velocity) {
	target.theta = round_angle(theta * PI/180);
	target.velocity = velocity;
	target.command = ORIENTATION_CONTROL;
	reset_timers();
}

void RobotController::start_velocity_control(float vel_left, float vel_right) {
	target.command = NO_CONTROL;
	controller.set_target_velocity(vel_left, vel_right, 1);
	reset_timers();
}

void RobotController::sensor_calibration() {
	#define sample_size_gyro_s 10000
	#define max_velocity 1.17f
	for (int i = 0; i < sample_size_gyro_s; ++i) {
		float v = max_velocity * i/sample_size_gyro_s;
		controller.set_target_velocity(-v, v, 1);
//		controller.set_target_velocity(-calibration_velocity,
//									   calibration_velocity, 1);
		Thread::wait(10);
	}
	sensors->wait = false;
	stop_and_wait();
}

void RobotController::start_calibration(float v) {
	calibration_velocity = v;
	sensors->wait = true;
	target.command = SENSOR_CALIBRATION;
	reset_timers();
}

void RobotController::reset_timers() {
	msg_timeout_timer.reset();
	msg_timeout_timer.start();
}

void RobotController::stop_and_wait() {
//	Flag tells Controller to stop robot
	controller.stop = true;
	vel_acelerada = 0;
	target.command = NO_CONTROL;
}

float RobotController::round_angle(float angle) {
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}

float RobotController::saturate(float value, float limit) {
	if(value > limit) value = limit;
	if(value < -limit) value = -limit;
	return value;
}

void RobotController::set_max_theta_error(float error) {
	max_theta_error = round_angle(error * PI/180);
}

bool RobotController::backwards_select(float target, float orientation) {
	if(backwards_timer.read_ms() > 25) {
		bool backwards = std::abs(round_angle(target - orientation)) > PI/2;
		if(previously_backwards != backwards) {
			backwards_timer.reset();
			vel_acelerada = 0.3;
		}
		previously_backwards = backwards;
		return backwards;
	} else {
		return previously_backwards;
	}
}
