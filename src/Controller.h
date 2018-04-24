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

		/**	@brief Wheel velocity control loop. Also updates odometry data */
		void control_loop();

		/**	@brief Sets PWM output for selected wheel.
		 *	@param w Wheel struct containing PwmOut object
		 *	@param pwm Desired PWM output. If abs(pwm) &lt 1, pwm is saturated to 1 or -1 */
		void set_pwm(wheel &w, float pwm);

		/**	@brief Computes velocity for both wheels using encoder data.
		 *	Also updates distance travelled, used for odometry */
		void update_wheel_velocity();

		/**	@brief Commputes PID output for selected wheel. Constants used are stored on PID struct.
		 *	Also saves current error and accumulated error on wheel struct, to be used on next iteration
		 *	@param w Wheel struct containing current velocity, last error and accumulated error
		 * 	used as imputs for controller
		 *	@return PID controller output */
		float get_pid_output(wheel& w);

		/**	@brief Sets velocity and PID data to 0.
		 *	@param w Wheel to reset */
		void reset(wheel &w);

		/**	@brief Stops both wheels, reset them and pauses control loop thread.
		 * 	Thread waits for signal CONTINUE_SIGNAL to resume */
		void stop_and_wait();

		/**	@brief Initializes PwmOut and QEI objects, used for motor control and odometry data
		 *	@param w Wheel struct. Pointer to PwmOut and QEI objects are stored there
		 *	@param tach_pin1 First pin connected to encoder
		 *	@param tach_pin2 Second pin connected to encoder
		 *	@param motor_pin1 First pin connected to motor
		 *	@param motor_pin2 Second pin connected to motor */
		void init_wheel(wheel &w, PinName tach_pin1, PinName tach_pin2,
						PinName motor_pin1, PinName motor_pin2);

	public:
		Thread control_thread;
		bool stop = true;

		PID pid = {1.26,0.0481,0};
		wheel left_wheel = {};
		wheel right_wheel = {};

		/**	@brief Constructor. Initializes left_wheel and right_wheel, creating PwmOut and QEI objects for them */
		Controller();

		/**	@brief Sets target velocity for both wheels, computed by (left * total) or (right * total)
		 *	@param left Desired left wheel velocity
		 *	@param right Desired right wheel velocity
		 *	@param total Desired total velocity for the robot */
		void set_target_velocity(float left, float right, float total);

		/**	@brief Sets PID constants for the wheel velocity controller, computed on get_pid_output
		 *	@param kp Constant multiplied by the error
		 *	@param ki Constant multiplied by the integrated error
		 *	@param kd Constant multiplied by the derivative of the error */
		void set_pid_constants(float kp, float ki, float kd);

		/**	@brief Sets signal CONTINUE_SIGNAL, resuming wheel velocity control thread */
		void continue_thread();

		/**	@brief Starts wheel velocity control thread */
		void start_thread();
};

#endif //VSSS_CONTROLLER2_H
