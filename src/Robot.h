#ifndef VSSS_ROBOT2_H
#define VSSS_ROBOT2_H

#include <mbed.h>
#include "Controller.h"
#include "Messenger.h"
#include <string>
class Messenger;

#define VECTOR_CONTROL 0
#define POSITION_CONTROL 1
#define ORIENTATION_CONTROL 2
#define NO_CONTROL 3

struct robot_state {
	float x;
	float y;
	float theta;
	float velocity;
	int command;
};

class Robot {
	private:
		Thread control_thread;
		Timer msg_timeout_timer;

		robot_state state = {};
		robot_state target = {};

		float vel_acelerada = 0;
		float orientation_Kp = 0.8;
		bool previously_backwards = false;

		void control_loop();
		void vector_control();
		void position_control();
		void orientation_control();
		void set_wheel_velocity_tan_controller(float theta_error, float velocity, bool backwards);
		void update_odometry();

		void stop_and_wait();
		void continue_threads();
		void reset_state();
		float round_angle(float angle);
		float saturate(float value, float limit);

	public:
		Messenger* messenger;
		Controller controller;

		float max_theta_error;
		float acc_rate;
		float kgz;
		int msg_timeout_limit;
		char MY_ID;

		explicit Robot(Messenger *msgr);
		void start_vector_control(float theta, float velocity, bool reset);
		void start_orientation_control(float theta, float velocity, bool reset);
		void start_position_control(float x, float y, float velocity, bool reset);
		void start_velocity_control(float vel_left, float vel_right);
		void set_max_theta_error(float error);
		void start_thread();
};

#endif //VSSS_ROBOT2_H
