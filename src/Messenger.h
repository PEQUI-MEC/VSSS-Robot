#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "RobotController.h"
#include "SensorFusion.h"
#include <string>
#include <array>
#include <sstream>

class RobotController;

struct Message {
	char type = ' ';
	int data_size = 0;
	std::array<float, 6> data;
};

class Messenger {

	public:
		char ID;
		XBeeLib::XBee802 xbee;

		bool battery_requested = false;

		float left_acm = 0;
		float right_acm = 0;
		int acm_count = 1;

		CircularBuffer<Message, 5> message_buffer;
		Message parse(const std::string &msg);
		void parse_and_add_to_buffer(const uint8_t *const data, uint16_t len);
		void update_by_messages(SensorFusion& sensors, RobotController& robot);
		void send_info(SensorFusion& sensors, RobotController& robot);

		void send_sensor_data(const std::string &msg);

		void process_xbee_msgs();

		/**	@brief Sends battery voltage. Example: "B7.53" */
		void send_battery();

	public:

		/**	@brief Constructor
		 *	@param id A unique ID assigned to each robot
		 *	@param xbee_addr Unique xbee address assigned to each robot **/
		Messenger(char id, uint16_t xbee_addr);

		/**	@brief Sends message to another xbee
		 *	@param msg Message to be sent
		 *	@param addr 16-bit address of the receiving xbee */
		void send_msg(const std::string &msg, uint16_t addr = 0x35D0);
};

#endif /* MESSENGER_H_ */
