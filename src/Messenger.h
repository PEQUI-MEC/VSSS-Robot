#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "Robot.h"
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

/**	@brief Sends data and receives commands by xbee. A command can set value to variables or start a controller.
 *
 * 	Messages can be sent through a std::ostream object, using the same syntax as std::cout:		<br>
 *		std::ostream m(messenger);																<br>
 *		m << "number: " << 123 << std::flush;	*/
class Messenger : public std::stringbuf {

	private:
		char ID;
		XBeeLib::XBee802 *xbee;

		Robot *robot;

		/**	@brief Decodes vector command message and starts vector controller.
		 *	@param msg Example: "V45;0.8" sets desired angle to 45 degrees and desired velocity to 0.8 m/s */
		void GoToVector(std::string msg);

		/**	@brief Decodes position command message and starts position controller.
		 *	@param msg Example: "P50;-50;0.8" sets desired position to (50,-50) desired velociy to 0.8 m/s */
		void GoToPoint(std::string msg);

		/**	@brief Decodes orientation command message and starts orientation controller.
		 *	@param msg Example: "O45;0.8" sets desired orientation to 45 degrees and desired orientation to 0.8 m/s */
		void goToOrientation(std::string msg);

		/**	@brief Decodes command for wheel velocity PID constants and sets kp, kd and ki constants on Controller.
		 *	@param msg Example: "K1;0.2;0.1" sets Kp to 1, Kd to 0.2 and Ki to 0.1 */
		void Update_PID_K(std::string msg);

		/**	@brief Decodes command for nonlinear controller constants and sets kgz and max_theta_error on Robot.
		 *	@param msg Example: "KP0.9;15" sets kgz to 0.9 and max_theta_error to 15 degrees */
		void Update_PID_Pos(std::string msg);

		/**	@brief Decodes command for nonlinear controller acceleration and sets acc_rate on Robot.
		 *	@param msg Example: "A2" sets desired acceleration to 2 m/s^2 */
		void Update_ACC(std::string msg);

		/**	@brief Sends battery voltage. Example: "B7.53" */
		void send_battery();

		/**	@brief Decodes old style message. Example: "A&#64O20;1#B&#64V45;0.8#" results in "V45;0.8" for the B robot
		 *	@param msg Message in old style. Supports messages for multiple robots
		 *	@return Message in the new style */
		std::string decode_strings(std::string msg);

		/**	@brief Breaks msg using ';' as delimiter, converts each substring to float and returns array with the result
		 *	@tparam size Expected number of substrings, also size of returned array
		 *	@param msg Input string, to be divided in substrings
		 *	@param first_char_pos Function ignores all chars before first_char_position
		 *	@return Struct containing output float array and flag is_valid,
		 *	set to false if number of substrings is smaller than expected */
		template<int size>
		msg_data<size> get_values(const std::string& msg, unsigned int first_char_pos);

	public:
		bool debug_mode;

		/**	@brief Constructor
		 *	@param id A unique ID assigned to each robot
		 *	@param robot Poiter to Robot, used to set constants and start controllers
		 *	@param this_xbee XBee802 object, used for sending and receiving messages */
		Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee);

		/**	@brief Decodes received message and executes command
		 *	@param msg Message containing command to be executed */
		void decode_msg(std::string msg);

		/**	@brief Sends message to another xbee
		 *	@param msg Message to be sent
		 *	@param addr 16-bit address of the receiving xbee */
		void send_msg(std::string msg, uint16_t addr = 0x35D0);

		/**	@brief Overrides std::stringbuf::sync().
		 *	Sends buffer content to default address after a flush is received (std::flush or std::endl)
		 *	@return Always returns 0 */
		int sync() override {
			send_msg(this->str());
			this->str("");
			return 0;
		}
};

#endif /* MESSENGER_H_ */