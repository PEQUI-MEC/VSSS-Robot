#include <string>
#include "Messenger.h"
#include "PIN_MAP.h"
#include "Control.h"
#include "ConfigFile.h"
#include "helper_functions.h"

#define PI 3.1415926f
using std::string;

void Messenger::send_msg(const string &msg, uint16_t addr) {
	XBeeLib::RemoteXBee802 remoteDevice = XBeeLib::RemoteXBee802(addr);
	xbee.send_data(remoteDevice, (const uint8_t *) msg.c_str(), (uint16_t ) msg.size(), true);
}

void Messenger::send_battery() {
	AnalogIn vin_all_cells(ALL_CELLS);
	float vbat = vin_all_cells.read() * (3.3f * 1470.0f/470.0f);
	float vbat_round = std::round(vbat*1000)/1000;
	string msg_bat = "B" + std::to_string(vbat_round);
	send_msg(msg_bat);
}

void Messenger::ekf_msg(const Message &msg) {
	control->set_ekf_vision_data(msg.get<float>(1),
								 msg.get<float>(5),
								 msg.get<float>(9));
//	send_msg(str(msg.get<float>(1)) + ", " +
//					 str(msg.get<float>(5)) + ", " + str(msg.get<float>(9)));
}

void Messenger::position_control_msg(const Message &msg) {
	control->set_target_position(msg.get<float>(1),
								 msg.get<float>(5),
								 msg.get<float>(9));
}

void Messenger::vector_control_msg(const Message &msg) {
	control->set_vector_control(msg.get<float>(1),
								msg.get<float>(5));
}

void Messenger::pose_control_msg(const Message &msg) {
	control->set_target_pose(msg.get<float>(1), msg.get<float>(5),
							 msg.get<float>(9), false, msg.get<float>(13));
}

void Messenger::orientation_control_msg(const Message &msg) {
	control->set_target_orientation(msg.get<float>(1));
}

void Messenger::angular_vel_control_msg(const Message &msg) {
	control->set_ang_vel_control(msg.get<float>(1));
}

void Messenger::decode_msg(const Message &msg) {
	switch (msg.get<uint8_t>(0)) {
		case 0 :
			ekf_msg(msg);
			break;
		case 1 :
			position_control_msg(msg);
			break;
		case 2 :
			vector_control_msg(msg);
			break;
		case 3 :
			pose_control_msg(msg);
			break;
		case 4 :
			orientation_control_msg(msg);
			break;
		case 5 :
			angular_vel_control_msg(msg);
			break;
		default:
			break;
	}
}

void Messenger::start_thread() {
	xbee_thread.start(callback(this, &Messenger::xbee_thread_callback));
}

void Messenger::xbee_thread_callback() {
	while (true) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
		xbee.process_rx_frames();
	}
}

Messenger * messenger_ptr;
Messenger::Messenger(Control *control)
		: xbee(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 115200) {

	setlocale(LC_ALL, "C");

	messenger_ptr = this;
	this->control = control;

	{
		ConfigFile configs("/local/config.txt");
		xbee_addr = configs.get_xbee_addr();
	}

	xbee.set_on_complete_callback([]() {
		messenger_ptr->xbee_thread.signal_set(CONTINUE_SIGNAL);
	});

	xbee.register_receive_cb([](const XBeeLib::RemoteXBee802 &remote,
								bool broadcast, const uint8_t * data, uint16_t len) {
		if (len != 0) {
			messenger_ptr->decode_msg({data, len});
		}
	});

	XBeeLib::RadioStatus const radioStatus = xbee.init();
	MBED_ASSERT(radioStatus == XBeeLib::Success);
	xbee.set_network_address(xbee_addr);
}

