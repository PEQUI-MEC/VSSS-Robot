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
#define UVF_CONTROL 4

struct robot_state {
	float x;
	float y;
	float theta;
	float velocity;
	int command;
	float ref_x;
	float ref_y;
};

class Robot {
	private:
		Thread control_thread;
		Timer msg_timeout_timer;

		robot_state state = {};
		robot_state target = {};

		float uvf_n = 2;
		float vel_acelerada = 0;
		float orientation_Kp = 0.8;
		bool previously_backwards = false;


		/**	@brief Main control loop. Calls vector_control, position_control or orientation_control
		 *	depending on state.command. Controller can be selected by calling start_vector_control,
		 *	start_position_control or start_orientation_control */
		void control_loop();

		/**	@brief Executes controller for the uvf command. Is started by start_uvf_control */
		void uvf_control();

		/**	@brief Executes controller for the vector command. Is started by start_vector_control */
		void vector_control();

		/**	@brief Executes controller for the position command. Is started by start_position_control */
		void position_control();

		/**	@brief Executes controller for the orientation command. Is started by start_orientation_control */
		void orientation_control();

		/**	@brief Sets the target velocity for each wheel using the output of the nonlinear controller
		 * 	@param theta_error Controller input. Difference between the target orientation and the current orientation.
		 * 	@param velocity Total desired velocity. Multiplies controller output for each wheel
		 * 	@param backwards True for backwards movement, false for forwards movement */
		void set_wheel_velocity_nonlinear_controller(float theta_error, float velocity, bool backwards);

		/**	@brief Updates position and orientation stored on Robot::state, using odometry data
		 *	computed by controller.update_wheel_velocity */
		void update_odometry();

		/**	@brief Pauses threads control_thread and controller.control_thread and waits for the signal CONTINUE_SIGNAL.
		 *	Signal is sent by continue_threads */
		void stop_and_wait();

		/**	@brief Sets signal CONTINUE_SIGNAL, resuming control_thread and controller.control_thread */
		void continue_threads();

		/** @brief Sets position and orientation to 0. Used on relative commands */
		void reset_state();

		/** @brief Angle is converted to a value between -PI and PI
		 *	@param angle Input angle in radians
		 *	@return Equivalent angle between -PI and PI, in radians */
		float round_angle(float angle);

		/**	@brief Saturates value, setting an upper and lower limit
		 *	@param value Floating point number to be saturated
		 *	@param limit Upper and lower limit
		 *	@return Returns value if (abs(value) &lt limit),
		 *	limit if (value > limit), or -limit if (value &lt -limit) */
		float saturate(float value, float limit);

	public:
		Messenger* messenger;
		Controller controller;

		float max_theta_error;
		float acc_rate;
		float kgz;
		int msg_timeout_limit;
		char MY_ID;

		/**	@brief Constructor
		 * 	@param msgr Pointer to Messenger, can be used to send logs */
		explicit Robot(Messenger *msgr);

		/**	@brief Executes vector command. Configures Robot::target used on the main control loop
		 *	@param x X component of the desired position
		 *	@param y Y component of the desired position
		 *	@param x_ref X component of the uvf reference
		 *	@param y_ref Y component of the uvf reference
		 *	@param n UVF constant, defines curvature
		 *	@param velocity Desired velocity
		 *	@param reset Executes a relative command if true, by setting state variables to 0 */
		void start_uvf_control(float x, float y, float x_ref, float y_ref, float n, float velocity, bool reset);

		/**	@brief Executes vector command. Configures Robot::target used on the main control loop
		 * 	@param theta Desired orientation
		 * 	@param velocity Desired velocity
		 * 	@param reset Executes a relative command if true, by setting state variables to 0 */
		void start_vector_control(float theta, float velocity, bool reset = true);

		/**	@brief Executes orientation command. Configures Robot::target used on the main control loop
		 *	@param theta Desired orientation
		 *	@param velocity Desired velocity */
		void start_orientation_control(float theta, float velocity, bool reset = true);

		/**	@brief Executes position command. Configures Robot::target used on the main control loop
		 *	@param x X component of the desired position
		 *	@param y Y component of the desired position
		 *	@param velocity Desired velocity
		 *	@param reset Executes a relative command if true, by setting state variables to 0 */
		void start_position_control(float x, float y, float velocity, bool reset = true);

		/**	@brief Executes velocity command. Sets robot.command to NO_CONTROL
		 *	@param vel_left Desired velocity for the left wheel
		 *	@param vel_right Desired velocity for the right wheel */
		void start_velocity_control(float vel_left, float vel_right);

		/**	@brief Setter for max_theta_error
		 *	@param error Maximum allowed theta_error, in degrees */
		void set_max_theta_error(float error);

		/**	@brief Starts main control loop thread */
		void start_thread();
};

#endif //VSSS_ROBOT2_H
