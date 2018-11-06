#include "mbed.h"
#include "XBeeLib.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "SensorFusion.h"
#include "helper_functions.h"
#include "ConfigFile.h"
#include <cmath>
#include <fstream>

#define PI 3.141592f

using std::string;

XBeeLib::XBee802 *xbee;
uint16_t xbee_addr;
Robot *robot = nullptr;
Messenger *messenger = nullptr;
Thread* t_rx;

void rx_thread() {
	while (true) {
		if(t_rx->get_state() != Thread::WaitingThreadFlag) {
			Thread::signal_wait(CONTINUE_SIGNAL);
			Thread::signal_clr(CONTINUE_SIGNAL);
		}
		xbee->process_rx_frames();
	}
}

static void receive_cb(const XBeeLib::RemoteXBee802 &remote, bool broadcast,
					   const uint8_t *const data, uint16_t len) {
	if (len != 0) {
		string msg = string((const char *) data, len);
		messenger->decode_msg(msg);
	}
}

static void process_frames() {
	t_rx->signal_set(CONTINUE_SIGNAL);
}

void led_write(std::array<DigitalOut, 4> &LEDs, uint8_t num) {
	LEDs[0] = ((num >> 0) & 1);
	LEDs[1] = ((num >> 1) & 1);
	LEDs[2] = ((num >> 2) & 1);
	LEDs[3] = ((num >> 3) & 1);
}

void bat_watcher(std::array<DigitalOut, 4> &LEDs, AnalogIn &battery_vin) {
	double vbat = battery_vin.read() * (3.3 * 1470 / 470);
	double threshold = (vbat - 6.6) / 1.4;

	if (threshold >= 0.75) led_write(LEDs, 0b1111);
	else if (threshold >= 0.5) led_write(LEDs, 0b0111);
	else if (threshold >= 0.25) led_write(LEDs, 0b0011);
	else led_write(LEDs, 0b0001);
}

SensorFusion* sensors;
int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	robot = new Robot();

	{
		ConfigFile configs("/local/config.txt");
		configs.configure(*robot, xbee_addr);
	}

	xbee = new XBeeLib::XBee802(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 115200);

	xbee->register_receive_cb(&receive_cb);

	XBeeLib::RadioStatus const radioStatus = xbee->init();
	MBED_ASSERT(radioStatus == XBeeLib::Success);
	xbee->set_network_address(xbee_addr);

	xbee->set_complete_callback(&process_frames);

	t_rx = new Thread;
	t_rx->start(&rx_thread); // Handle de erro na thread da serial
//	t_rx.set_priority(osPriorityHigh);

	robot->controller.set_target_velocity(0,0,0);

	sensors = new SensorFusion(&robot->controller);
	robot->sensors = sensors;

	messenger = new Messenger(robot->MY_ID, robot, xbee, sensors);

	robot->start_thread();

//	Serial usb(USBTX, USBRX);

	robot->start_orientation_control(0, 0.8);
	wait(0.1);
	robot->start_orientation_control(-45, 0.8);
	wait(0.5);
	robot->start_orientation_control(0, 0.8);
	wait(0.5);
	robot->start_orientation_control(45, 0.8);
	wait(0.5);
	robot->start_orientation_control(0, 0.8);
	wait(0.5);

	while (true) {
		bat_watcher(LEDs, battery_vin);
		if (messenger->debug_mode) {
//			Utilizado para eviar dados p/ PC utilizando Messenger
		}
		Thread::wait(200);
//		std::string msg = std::to_string(sensors->prev_mesure.gyro_w) + '\n';
//		usb.printf(msg.c_str());
	}
}
