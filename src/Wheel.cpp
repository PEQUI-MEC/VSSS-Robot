#include "Wheel.h"

#include "PIN_MAP.h"
#include <cmath>

#define PI 3.1415926f
#define CONTROL_LOOP_MS 10

void WheelEncoder::update_wheel_velocity() {
	int time_us = timer.read_us();
	if(time_us != 0) {
		int pulses = encoder.getPulses();
		encoder.reset();
		timer.reset();
		float time = time_us/1E6f; // Time in seconds
		float previous_velocity = velocity;
//		0.06f*PI: m/s conversion
		velocity = (pulses*0.06f*PI)/(PULSES_PER_REVOLUTION * MOTOR_REVOLUTION_PER_WHEEL_REV * time);
		acceleration = velocity - previous_velocity;
	}
}

void WheelEncoder::reset() {
	encoder.reset();
	timer.reset();
	velocity = 0;
	acceleration = 0;
}

float PID::get_output(float velocity, float target_velocity) {
	float error = target_velocity - velocity;
	float error_deriv = error - last_error;
	error_acc += error;
	return error * kp + error_acc * ki + error_deriv * kd;
}

void PID::reset() {
	error_acc = 0;
	last_error = 0;
}

void PID::set_constants(float kp, float ki, float kd) {
	kp = kp;
	ki = ki;
	kd = kd;
}

void WheelController::set_pwm_by_pid(float velocity, float target_velocity) {
	set_pwm(target_velocity);
	// float pid_output = pid.get_output(velocity, target_velocity);
	// set_pwm(pid_output);
}

void WheelController::set_pwm(float pwm) {
	if(pwm > 1) pwm = 1;
	if(pwm < -1) pwm = -1;
	if(std::abs(pwm) < 0.05) pwm = 0;

	if (pwm < 0) {
		pwm_out1.write(1);
		pwm_out2.write(1 + pwm);
	} else if (pwm > 0) {
		pwm_out1.write(1 - pwm);
		pwm_out2.write(1);
	} else {
		pwm_out1.write(1);
		pwm_out2.write(1);
	}
}

void WheelController::stop_and_reset() {
	set_pwm(0);
	pid.reset();
}

void WheelController::set_pid_constants(float kp, float ki, float kd) {
	pid.set_constants(kp, ki, kd);
	pid.reset();
}
