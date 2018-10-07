#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "SensorFusion.h"
#include "Control.h"
#include <string>
#include <array>
#include "helper_functions.h"

template<int size>
struct msg_data {
	std::array<float, size> data;
	bool is_valid;

	/**	@brief Operator overload, simplifies access to data array
	 *	@param i Data array index
	 *	@return Float value stoered in data */
	inline float &operator[](int i) {
		return data[i];
	}
};

class Messenger {

	private:
	XBeeLib::XBee802 xbee;
	Thread xbee_thread;
	uint16_t xbee_addr;

	Control *control;

	void xbee_thread_callback();

	/**	@brief Sends battery voltage. Example: "B7.53" */
	void send_battery();

	/**	@brief Sets new vision data (x, y and theta) for a new EKF update
	 *	@param msg Example: "E20;50;10" sets measured position to {20,50}cm and orientation to 10 degrees */
	void set_ekf_data(const std::string &msg);

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

	/**	@brief Decodes received message and executes command
		 *	@param msg Message containing command to be executed */
	void decode_msg(const std::string &msg);

	/**	@brief Breaks msg using ';' as delimiter, converts each substring to float and returns array with the result
	 *	@tparam size Expected number of substrings, also size of returned array
	 *	@param msg Input string, to be divided in substrings
	 *	@param first_char_pos Function ignores all chars before first_char_position
	 *	@return Struct containing the resulting float array and flag is_valid,
	 *	set to false if number of substrings is smaller than expected */
	template<int size>
	msg_data<size> get_values(const std::string &msg, unsigned int first_char_pos);

	public:
	/**	@brief Constructor
	 *	@param id A unique ID assigned to each robot
	 *	@param robot Pointer to Robot, used to set constants and start controllers
	 *	@param this_xbee XBee802 object, used for sending and receiving messages
	 *	@param sensors_ptr Pointer to SensorFusion, used to set new data for an EKF vision update*/
	explicit Messenger(Control *control);

	/**	@brief Sends message to another xbee
	 *	@param msg Message to be sent
	 *	@param addr 16-bit address of the receiving xbee */
	void send_msg(const std::string &msg, uint16_t addr = 0x35D0);

	void start_thread();

//	Sends csv logs
	template <typename ...T>
	void send_log(T ...data) {
		std::string msg;
		append(msg, data...);
		send_msg(msg);
	}
};

#endif /* MESSENGER_H_ */
