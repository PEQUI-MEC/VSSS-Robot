#include "Wheel.h"

#include "Controller.h"
#include "PIN_MAP.h"
#include <cmath>

#define PI 3.1415926f
#define CONTROL_LOOP_MS 10

void WheelController::pwm_control_loop() {
	float pid_left  = pid.get_pid_output(target_velocity);
	set_pwm(left_wheel, pid_left);
}

void WheelController::set_pwm(float pwm) {
	if(pwm > 1) pwm = 1;
	if(pwm < -1) pwm = -1;
	if(std::abs(pwm) < 0.05) pwm = 0;

	if (pwm < 0) {
		pwm_out1->write(1);
		pwm_out2->write(1 + pwm);
	} else if (pwm > 0) {
		pwm_out1->write(1 - pwm);
		pwm_out2->write(1);
	} else {
		pwm_out1->write(1);
		pwm_out2->write(1);
	}
}

float PID::get_output(float velocity, float target_velocity) {
	float error = target_velocity - velocity;
	float error_deriv = error - last_error;
	error_acc += error;
	return error * kp + error_acc * ki + error_deriv * kd;
}

void Controller::update_wheel_velocity() {
	int left_pulses = left_wheel.encoder->getPulses();
	int right_pulses = right_wheel.encoder->getPulses();
	float time = timer.read_us()/1E6f; // Time in seconds

	left_wheel.encoder->reset();
	right_wheel.encoder->reset();
	timer.reset();

	if(time != 0) {
//		0.06f*PI: m/s conversion
		float last_left_vel = left_wheel.velocity;
		float last_right_vel = right_wheel.velocity;
		left_wheel.velocity = (left_pulses*0.06f*PI)/(PULSES_PER_REVOLUTION * MOTOR_REVOLUTION_PER_WHEEL_REV * time);
		right_wheel.velocity = (right_pulses*0.06f*PI)/(PULSES_PER_REVOLUTION * MOTOR_REVOLUTION_PER_WHEEL_REV * time);
		encoder_vel = {true, left_wheel.velocity, right_wheel.velocity,
				 left_wheel.velocity - last_left_vel, right_wheel.velocity - last_right_vel};
	}
}

void Controller::set_target_velocity(float left, float right, float total) {
	left_wheel.target_velocity = left * total;
	right_wheel.target_velocity = right * total;
};

void Controller::stop_and_wait() {
	set_target_velocity(0, 0, 0);
	set_pwm(left_wheel, 0);
	set_pwm(right_wheel, 0);
	reset(left_wheel);
	reset(right_wheel);
	stop = false;
}

void Controller::init_wheel(wheel& w, PinName tach_pin1, PinName tach_pin2, PinName motor_pin1, PinName motor_pin2) {
	w.encoder = new QEI(tach_pin1, tach_pin2, NC, PULSES_PER_REVOLUTION, QEI::X4_ENCODING);
	w.pwm_out1 = new PwmOut(motor_pin1);
	w.pwm_out1->period_ms(2);
	w.pwm_out2 = new PwmOut(motor_pin2);
	w.pwm_out2->period_ms(2);
	w.error_acc = 0;
}

void Controller::set_pid_constants(float kp, float ki, float kd) {
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	reset(right_wheel);
	reset(left_wheel);
}

void Controller::reset(wheel& w) {
	w.velocity = 0;
	w.error_acc = 0;
	w.last_error = 0;
}
