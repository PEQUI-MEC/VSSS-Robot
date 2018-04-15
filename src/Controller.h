#ifndef VSSS_CONTROLLER2_H
#define VSSS_CONTROLLER2_H

#include "QEI.h"

#define CONTINUE_SIGNAL 1

struct wheel {
	QEI* encoder;
	PwmOut* pwm_out1;
	PwmOut* pwm_out2;
	float velocity;
	float target_velocity;
	float error_acc;
	float last_error;
	float encoder_distance;
};

struct PID {
	float kp;
	float ki;
	float kd;
};

class Controller {
	private:
		Timer timer;

		void control_loop();
		void set_pwm(wheel &w, float pwm);
		void update_wheel_velocity();
		float get_pid_output(wheel& w);
		void reset(wheel &w);
		void stop_and_wait();
		void init_wheel(wheel &w, PinName tach_pin1, PinName tach_pin2,
						PinName motor_pin1, PinName motor_pin2);

	public:
		Thread control_thread;
		bool stop = true;

		PID pid = {1.26,0.0481,0};
		wheel left_wheel = {};
		wheel right_wheel = {};

		Controller();
		void set_target_velocity(float left, float right, float total);
		void set_pid_constants(float kp, float ki, float kd);
		void continue_thread();
		void start_thread();
};

#endif //VSSS_CONTROLLER2_H
