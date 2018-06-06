//IMPORTS
#include "mbed.h"
#include "XBeeLib.h"
#include "Messenger.h"
#include "IMU.h"
#include "PIN_MAP.h"
#include "SensorFusion.h"
#include <cmath>
#include <fstream>

#define PI 3.141592f

using std::string;

XBeeLib::XBee802 *xbee;
uint16_t addr;

//MANIPULADOR DE MENSAGENS
Robot *robot = nullptr;
Messenger *messenger = nullptr;

//SERIAL THREAD*****************************************************************
void rx_thread() {
	while (true) {
		xbee->process_rx_frames();
		Thread::wait(20);
	}
}

void set_config(const string &type, const string &data,
				Robot &robot, uint16_t &xbee_addr) {
	if(type == "addr") {
		xbee_addr = (uint16_t) std::stoul(data, nullptr, 16);
	} else if(type == "my_id") {
		robot.MY_ID = data[0];
	} else if(type == "msg_timeout") {
		robot.msg_timeout_limit = std::stoi(data);
	} else if(type == "acc_rate") {
		robot.acc_rate = std::stof(data);
	} else if(type == "kgz") {
		robot.kgz = std::stof(data);
	} else if(type == "max_theta_error") {
		robot.max_theta_error = std::stof(data);
	}
}

void configure_by_file(const string &path, int file_size,
					   Robot &robot, uint16_t &xbee_addr) {
	LocalFileSystem local("local");
	FILE *config_file = fopen(path.c_str(), "r");
	for (int i = 0; i < file_size; ++i) {
		char * cline;
		size_t size;
		if(__getline(&cline, &size, config_file) == -1) return;
		string line = string(cline, size);
		size_t separator = line.find(':');
		set_config(line.substr(0, separator), line.substr(separator + 1), robot, xbee_addr);
	}
	fclose(config_file);
}

//void escrever_arquivo() {
//	FILE *fp = fopen("/local/config.txt", "w");
//	fprintf(fp, "addr:%04x\n", addr);
//	fprintf(fp, "my_id:%c\n", robot->MY_ID);
//	fprintf(fp, "msg_timeout:%d\n", robot->MSG_TIMEOUT);
//	fprintf(fp, "acc_rate:%f\n", robot->ACC_RATE);
//	fprintf(fp, "kgz:%lf\n", robot->kgz);
//	fprintf(fp, "max_theta_error:%f\n", robot->MAX_Theta_Error);
//	fclose(fp);
//}

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

static void receive_cb(const XBeeLib::RemoteXBee802 &remote, bool broadcast,
					   const uint8_t *const data, uint16_t len) {
	if (len != 0) {
		string msg = string((const char *) data, len);
		messenger->decode_msg(msg);
	}
}

void mag_calibration() {
	IMU imu{};
	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
	Thread::wait(1000);

	#define sample_size 5000
	for (int i = 0; i < sample_size; ++i) {
		robot->start_velocity_control(-0.05f, 0.05f);
		auto data = imu.read_mag_components();
		string msg = std::to_string(data.x) + ',' + std::to_string(data.y);
		messenger->send_msg(msg);
		Thread::wait(10);
	}
}

float gyro_calib() {
	IMU imu{};
	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
	float acc = 0;

	#define sample_size_gyro 500
	for (int i = 0; i < sample_size_gyro; ++i) {
		acc += imu.read_gyro();
		wait_ms(5);
	}
	return acc/sample_size_gyro;
}

SensorFusion* sensors;
int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	robot = new Robot();
	configure_by_file("/local/config.txt", 6, *robot, addr);

	xbee = new XBeeLib::XBee802(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 115200);

	xbee->register_receive_cb(&receive_cb);

	XBeeLib::RadioStatus const radioStatus = xbee->init();
	MBED_ASSERT(radioStatus == XBeeLib::Success);
	xbee->set_network_address(addr);

	Thread t_rx;
	t_rx.start(rx_thread); // Handle de erro na thread da serial
	t_rx.set_priority(osPriorityHigh);

	robot->controller.set_target_velocity(0,0,0);
	float offset = gyro_calib();

	sensors = new SensorFusion(&robot->controller);
	sensors->gyro_offset = offset;
	robot->sensors = sensors;
	sensors->ekf_thread_start();

	messenger = new Messenger(robot->MY_ID, robot, xbee, sensors);

	robot->start_thread();

	robot->start_orientation_control(0, 0.8);
	wait(0.1);
	robot->start_orientation_control(-45, 0.8);
	wait(0.5);
	robot->start_orientation_control(0, 0.8);
	wait(0.5);
	robot->start_orientation_control(45, 0.8);
	wait(0.5);
	robot->start_orientation_control(0, 0.8);

	while (true) {
		bat_watcher(LEDs, battery_vin);
		if (messenger->debug_mode) {
//			Utilizado para eviar dados p/ PC utilizando Messenger
		}
		Thread::wait(1000);
	}
}
