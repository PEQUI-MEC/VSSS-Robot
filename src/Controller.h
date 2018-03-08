#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "mbed.h"
#include "QEI.h"

class Controller {
	private:

		int timeout;
		float PID;
		int pulses_per_revolution;
		float DT;
		float accErr;
		int Control_DT;
		void Tach_Increment();

	public:
		float *acc_pos;
		float kp, ki, kd;
		float *target_vel;
		float currErr, lastVel;
		float currVel;
		float PWM;
		bool debug_mode;
		Timer msg_timeout;
		Timer timer;
		Timer sampling_timer;
		QEI *wheel;

		float get_vel();
		void resetPos();
		void set_PID_constants(float KP, float KI, float KD);
		void set_target_velocity(float desired_vel);
		void
		init(float *acc_position, float *desiredVel, PinName INTERRUPT_PIN_1, PinName INTERRUPT_PIN_2, int PR, int TO,
			 int CL);
		void control_loop();
		void reset();
		void start_timer_new_msg();
};

#endif /* CONTROLLER_H_ */
