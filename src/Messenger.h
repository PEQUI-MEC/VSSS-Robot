#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "SensorFusion.h"
#include "Control.h"
#include <string>
#include <array>

struct Message {
	const uint8_t * const data;
	uint16_t length;

	template<typename T>
	inline const T get(uint32_t offset) const {
		return *reinterpret_cast<const T *>(data + offset);
	}
};

class Messenger {

	private:
		XBeeLib::XBee802 xbee;
		Thread xbee_thread;
		uint16_t xbee_addr;

		Control *control;

		void xbee_thread_callback();
		void decode_msg(const Message &msg);
		void ekf_msg(const Message &msg);
		void pose_control_msg(const Message &msg);

		/**	@brief Sends battery voltage. Example: "B7.53" */
		void send_battery();

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
		void vector_control_msg(const Message &msg);
		void position_control_msg(const Message &msg);
		void orientation_control_msg(const Message &msg);
		void angular_vel_control_msg(const Message &msg);
};

#endif /* MESSENGER_H_ */
