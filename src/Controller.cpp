#include "Controller.h"
#include "PIN_MAP.h"
#include <cmath>

#define PI 3.1415926f
#define CONTROL_LOOP_MS 10

Controller::Controller() {
	init_wheel(left_wheel, ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2, MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2);
	init_wheel(right_wheel, ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2);
}

void Controller::start_thread() {
	timer.start();
	control_thread.start(callback(this, &Controller::control_loop));
}

void Controller::control_loop() {
	while (true) {
		update_wheel_velocity();

		float pid_left  = get_pid_output(left_wheel);
		float pid_right = get_pid_output(right_wheel);

		set_pwm(left_wheel, pid_left);
		set_pwm(right_wheel, pid_right);

//		Stop flag is set on Robot after a timeout or arriving at destination
		if(stop) stop_and_wait();
		Thread::wait(CONTROL_LOOP_MS);
	}
}

void Controller::set_pwm(wheel &w, float pwm) {
	if(pwm > 1) pwm = 1;
	if(pwm < -1) pwm = -1;
	if(std::abs(pwm) < 0.05) pwm = 0;
//	pwm = 0;

	if (pwm < 0) {
		w.pwm_out1->write(1);
		w.pwm_out2->write(1 + pwm);
	} else if (pwm > 0) {
		w.pwm_out1->write(1 - pwm);
		w.pwm_out2->write(1);
	} else {
		w.pwm_out1->write(1);
		w.pwm_out2->write(1);
	}
}

float Controller::get_pid_output(wheel& w) {
	float error = w.target_velocity - w.velocity;
	float error_deriv = error - w.last_error;
	w.error_acc += error;
	return error * pid.kp + w.error_acc * pid.ki + error_deriv * pid.kd;
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
		left_wheel.velocity = (left_pulses*0.05f*PI)/(PULSES_PER_REVOLUTION * MOTOR_REVOLUTION_PER_WHEEL_REV * time);
		right_wheel.velocity = (right_pulses*0.05f*PI)/(PULSES_PER_REVOLUTION * MOTOR_REVOLUTION_PER_WHEEL_REV * time);
		encoder_vel = {true, left_wheel.velocity, right_wheel.velocity,
				 left_wheel.velocity - last_left_vel, right_wheel.velocity - last_right_vel};
	}
}

void Controller::set_target_velocity(float left, float right, float total) {
	left_wheel.target_velocity = left * total;
	right_wheel.target_velocity = right * total;
};

void Controller::set_target_velocity(WheelVelocity target_velocity) {
	left_wheel.target_velocity = target_velocity.left;
	right_wheel.target_velocity = target_velocity.right;
}

void Controller::continue_thread() {
	if(control_thread.get_state() == Thread::WaitingThreadFlag) {
		control_thread.signal_set(CONTINUE_SIGNAL);
	}
}

void Controller::stop_and_wait() {
	set_target_velocity(0, 0, 0);
	set_pwm(left_wheel, 0);
	set_pwm(right_wheel, 0);
	reset(left_wheel);
	reset(right_wheel);
	stop = false;
	if(control_thread.get_state() != Thread::WaitingThreadFlag) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
	}
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
