#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "Robot.h"
#include "SensorFusion.h"
#include <string>
#include <array>
#include <sstream>

class Robot;

template <int size>
struct msg_data {
	std::array<float, size> data;
	bool is_valid;

	/**	@brief Operator overload, simplifies access to data array
	 *	@param i Data array index
	 *	@return Float value stoered in data */
	inline float& operator[](int i) {
		return data[i];
	}
};

class Messenger {

	private:
		char ID;
		XBeeLib::XBee802 *xbee;

		Robot *robot;
		SensorFusion *sensors;

		/**	@brief Sets new vision data (x, y and theta) for a new EKF update
		 *	@param msg Example: "E20;50;10" sets measured position to {20,50}cm and orientation to 10 degrees */
		void set_ekf_data(const std::string &msg);

		/**	@brief Sends current estimated pose to default xbee address */
		void send_information();

		/**	@brief Decodes uvf command message and starts uvf controller
		 *	@param msg Example: "U20;30;40;45;2;0.8" sets target position to {20,30}, with uvf reference to {40,45},
		 *	using 2 as calibration constant n and desired velocity to 0.8m/s */
		void uvf_message(const std::string &msg);

		/**	@brief Decodes vector command message and starts vector controller.
		 *	@param msg Example: "V45;0.8" sets desired angle to 45 degrees and desired velocity to 0.8 m/s */
		void GoToVector(const std::string &msg);

		/**	@brief Decodes position command message and starts position controller.
		 *	@param msg Example: "P50;-50;0.8" sets desired position to (50,-50) desired velociy to 0.8 m/s */
		void GoToPoint(const std::string &msg);

		/**	@brief Decodes orientation command message and starts orientation controller.
		 *	@param msg Example: "O45;0.8" sets desired orientation to 45 degrees and desired orientation to 0.8 m/s */
		void goToOrientation(const std::string &msg);

		/**	@brief Decodes command for wheel velocity PID constants and sets kp, kd and ki constants on Controller.
		 *	@param msg Example: "K1;0.2;0.1" sets Kp to 1, Kd to 0.2 and Ki to 0.1 */
		void Update_PID_K(std::string msg);

		/**	@brief Decodes command for nonlinear controller constants and sets kgz and max_theta_error on Robot.
		 *	@param msg Example: "KP0.9;15" sets kgz to 0.9 and max_theta_error to 15 degrees */
		void Update_PID_Pos(std::string msg);

		/**	@brief Decodes command for nonlinear controller acceleration and sets acc_rate on Robot.
		 *	@param msg Example: "A2" sets desired acceleration to 2 m/s^2 */
		void Update_ACC(std::string msg);

		void send_sensor_data(const std::string &msg);

		/**	@brief Sends battery voltage. Example: "B7.53" */
		void send_battery();

		/**	@brief Decodes old style message. Example: "A&#64O20;1#B&#64V45;0.8#" results in "V45;0.8" for the B robot
		 *	@param msg Message in old style. Supports messages for multiple robots
		 *	@return Message in the new style */
		std::string decode_strings(const std::string &msg);

		/**	@brief Breaks msg using ';' as delimiter, converts each substring to float and returns array with the result
		 *	@tparam size Expected number of substrings, also size of returned array
		 *	@param msg Input string, to be divided in substrings
		 *	@param first_char_pos Function ignores all chars before first_char_position
		 *	@return Struct containing the resulting float array and flag is_valid,
		 *	set to false if number of substrings is smaller than expected */
		template<int size>
		msg_data<size> get_values(const std::string& msg, unsigned int first_char_pos);

	public:
		bool debug_mode;

		/**	@brief Constructor
		 *	@param id A unique ID assigned to each robot
		 *	@param robot Pointer to Robot, used to set constants and start controllers
		 *	@param this_xbee XBee802 object, used for sending and receiving messages
		 *	@param sensors_ptr Pointer to SensorFusion, used to set new data for an EKF vision update*/
		Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee, SensorFusion *sensors_ptr);

		/**	@brief Decodes received message and executes command
		 *	@param msg Message containing command to be executed */
		void decode_msg(std::string msg);

		/**	@brief Sends message to another xbee
		 *	@param msg Message to be sent
		 *	@param addr 16-bit address of the receiving xbee */
		void send_msg(const std::string &msg, uint16_t addr = 0x35D0);

		// Appends csv data
		template <typename T>
		void append(std::string &buff, T last) {
			buff.append(std::to_string(last));
			//	buff += '\n';
		}

		template <typename First, typename ...Others>
		void append(std::string &buff,
					First first, Others ...others) {
			append(buff, first);
			buff += ',';
			append(buff, others...);
		}

		//	Sends csv logs
		template <typename ...T>
		void send_log(T ...data) {
			std::string msg;
			append(msg, data...);
			send_msg(msg);
		}
};

#endif /* MESSENGER_H_ */
