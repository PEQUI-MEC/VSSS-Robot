#include "VFOControl.h"
#include <cmath>
#include <tuple>

#define PI 3.1415926f

void VFOControl::vfo_control_loop() {
	while (true) {
		pose_data pose = sensors->get_pose();
		pose_vector pose_v{{pose.x, pose.y}, pose.theta, pose.v};
		vec u = get_controls({target.x, target.y}, target.theta, pose_v);
		float u1 = u.x, u2 = u.y;
		controller.set_target_velocity(u2 - u1*(ROBOT_SIZE/2.0f), u2 + u1*(ROBOT_SIZE/2.0f), 1);
		Thread::wait(10);
	}
}

vec VFOControl::get_controls(vec target_p, float target_theta, pose_vector &pose) {
	auto convergence_field = get_vector_field(target_p, target_theta, pose);
	vec &hp = std::get<0>(convergence_field);
	vec &hp_dot = std::get<1>(convergence_field);

	float fi_a;
	if(first_fi_a) {
		fi_a = std::atan2(sgnk * hp.y, sgnk * hp.x);
		previous_fi_a = fi_a;
		first_fi_a = false;
	} else {
		fi_a = std::atan2(sgnk * hp.y, sgnk * hp.x);
	}


	float fi_a_dot;
	if(hp.x == 0 && hp.y == 0) fi_a_dot = 0;
	else fi_a_dot = (hp_dot.y * hp.x - hp.y * hp_dot.x)/(hp.x * hp.x + hp.y * hp.y);
	float theta_error = F(fi_a - pose.theta);

	float u1 = k1 * theta_error + fi_a_dot;
	float u2 = get_g2(pose.theta) * hp;
	return {u1, u2};
}

float VFOControl::atan2c(float g3, float g2) {
	float fi = std::atan2(g3, g2);
	float fi_a_prev = F(previous_fi_a);
	float delta = fi - fi_a_prev;

	if(delta > PI) delta -= 2 * PI;
	else if (delta < -PI) delta += 2 * PI;
	fi = previous_fi_a + delta;
	previous_fi_a = fi;
	return fi;
}

std::tuple<vec, vec> VFOControl::get_vector_field(vec target_p, float target_theta, pose_vector &pose) {
	vec position_error = target_p - pose.position;

	float sigma = get_sigma(position_error);
	vec target_g2 = get_g2(target_theta);
	vec q_vt = target_g2 * sigma;

	vec hp = position_error * kp + q_vt;
	vec hp_dot = get_hp_dot(target_g2, sigma, pose);
	return {hp, hp_dot};
}

float VFOControl::get_sigma(vec error) {
	return -sgnk * n * error.vec_norm();
}

void VFOControl::set_sgnk(float target_x, float x) {
	float e20 = target_x - x;
	sgnk = (e20 > 0) - (e20 < 0);
}

vec VFOControl::get_hp_dot(vec &target_g2, float sigma, pose_vector &pose) {
	if(first_sigma) {
		previous_sigma = sigma;
		first_sigma = false;
		return {0,0};
	}
	vec vel = get_g2(pose.theta) * pose.v;
	float sig_dot = sigma - previous_sigma;
	previous_sigma = sigma;
	return vel * -kp + target_g2 * sig_dot;
}

vec VFOControl::get_g2(float theta) {
	return {std::cos(theta), std::sin(theta)};
}

float VFOControl::F(float angle) {
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}

VFOControl::VFOControl() {
	controller.set_target_velocity(0, 0, 0);
	target = {0,0,0};
}

void VFOControl::start_thread() {
	controller.start_thread();
	vfo_thread.start(callback(this, &VFOControl::vfo_control_loop));
}

