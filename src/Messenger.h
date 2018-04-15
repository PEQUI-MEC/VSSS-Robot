#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "Robot.h"
#include <string>
class Robot;

class Messenger {

	private:
		char ID;
		XBeeLib::XBee802 *xbee;
		std::string tokens[3];

		Robot *robot;
		void GoToVector(std::string msg);
		void Update_PID_K(std::string msg);
		void Update_PID_Pos(std::string msg);
		void GoToPoint(std::string msg);
		void Update_ACC(std::string msg);
		void goToOrientation(std::string msg);
		void send_battery();
		std::string decode_strings(std::string msg);
		bool get_tokens(std::string &msg, int size, unsigned int first_char_position);

	public:
		Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee);
		void decode_msg(std::string msg);
		void send_msg(std::string msg, uint16_t addr = 0x35D0);
		bool debug_mode;
};

#endif /* MESSENGER_H_ */