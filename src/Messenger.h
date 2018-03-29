#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "Controller.h"
#include "Robot.h"
#include "XBeeLib.h"
#include <string>
class Robot;

class Messenger {

	private:
		char ID;
		XBeeLib::XBee802 *xbee;
		std::string tokens[3];

		Robot *r;
		void GoToVector(std::string msg);
		void Update_PID_K(std::string msg);
		void Update_PID_Pos(std::string msg);
		void GoToPoint(std::string msg);
		void Update_ACC(std::string msg);
		void goToOrientation(std::string msg);
		void send_battery();
		std::string decode_strings(std::string msg);
		bool get_tokens(std::string& msg, int size);

	public:
		Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee);
		void decode_msg(std::string msg);
		void send_msg(std::string msg, uint16_t addr = 0x35D0);
		float ACC_RATE;
		bool bat_request;
		bool debug_mode;
		bool reinforcement_learning_mode;
		float desiredVel[2];
		float KPID[3];
		bool goToActive;
};

#endif /* MESSENGER_H_ */