#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include "Robot.h"
#include <string>
#include <array>

class Robot;

template <int size>
struct msg_data {
	std::array<float, size> data;
	bool is_valid;

	inline float& operator[](int i) {
		return data[i];
	}
};

class Messenger {

	private:
		char ID;
		XBeeLib::XBee802 *xbee;

		Robot *robot;
		void GoToVector(std::string msg);
		void Update_PID_K(std::string msg);
		void Update_PID_Pos(std::string msg);
		void GoToPoint(std::string msg);
		void Update_ACC(std::string msg);
		void goToOrientation(std::string msg);
		void send_battery();
		std::string decode_strings(std::string msg);

		template<int size>
		msg_data<size> get_values(const std::string& msg, unsigned int first_char_pos);

	public:
		bool debug_mode;
		Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee);
		void decode_msg(std::string msg);
		void send_msg(std::string msg, uint16_t addr = 0x35D0);
		void operator<<(const std::string &msg);
		void operator<<(float value);
};

#endif /* MESSENGER_H_ */