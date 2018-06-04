//IMPORTS
#include "mbed.h"
#include "XBeeLib.h"
#include "Messenger.h"
#include "IMU.h"
#include "PIN_MAP.h"
#include <cmath>
#include <fstream>

#if defined(ENABLE_LOGGING)
#include "DigiLoggerMbedSerial.h"
using namespace DigiLog;
#endif

#define PI 3.141592653589793238

using std::string;
using mbed::DigitalOut;
using mbed::AnalogIn;
using mbed::Serial;
using mbed::LocalFileSystem;
using XBeeLib::XBee802;
using XBeeLib::RemoteXBee802;
using XBeeLib::RadioStatus;

//*****************ENCODER HANDLER AND PID CONTROL FUNCTIONS******************//

//LEDS DO MBED
DigitalOut *led1, *led2, *led3, *led4;

//ANALOG INPUT MBED

AnalogIn *vin_all_cells;
AnalogIn *vin_single_cell;

//VARIAVEIS PARA GUARDAR LEITURA DO IMU
//int16_t *Mag;
//int16_t *Acc;
//int16_t *Gyro;

//DECLARANDO PWM DOS MOTORES
//IMU
//IMU* imu;
XBee802 *xbee;
uint16_t addr;
//SERIAL USB
Serial *log_serial;

//MANIPULADOR DE MENSAGENS
Robot *robot = nullptr;
Messenger *messenger = nullptr;

//VARIAVEIS DO CONTROLADOR DE POSICAO

double toDegrees(double rad) {
	return rad * 180 / PI;
}

//SERIAL THREAD*****************************************************************
void rx_thread() {
	while (true) {
		xbee->process_rx_frames();
		Thread::wait(20);
	}
}

//Timer t;
//
//double gyro_offset[3];
//double angulo_z = 0;
//
//void gyro_thread() {
//	while (1) {
//		imu->read_gyro(Gyro);
//		angulo_z += (((double) Gyro[2] * 0.00747703) - gyro_offset[2]) * (t.read_ms() / 1000.0);
//		t.reset();
//		t.start();
//		Thread::wait(1);
//	}
//}

string prox_string(FILE *fp, char delim) {
	string buffer;
	char ch = fgetc(fp);
	while (ch != delim && ch != '\0') {
		buffer += ch;
		ch = fgetc(fp);
	}
	return buffer;
}

LocalFileSystem *local;

void ler_arquivo() {
	FILE *fp = fopen("/local/config.txt", "r");
	if (prox_string(fp, ':') == "addr")
		addr = (uint16_t) strtol(prox_string(fp, '\n').c_str(), NULL, 16);
	if (prox_string(fp, ':') == "my_id")
		robot->MY_ID = prox_string(fp, '\n')[0];
	if (prox_string(fp, ':') == "msg_timeout")
		robot->msg_timeout_limit = atoi(prox_string(fp, '\n').c_str());
	if (prox_string(fp, ':') == "acc_rate")
		robot->acc_rate = float(atof(prox_string(fp, '\n').c_str()));
	if (prox_string(fp, ':') == "kgz")
		robot->kgz = float(atof(prox_string(fp, '\n').c_str()));
	if (prox_string(fp, ':') == "max_theta_error")
		robot->set_max_theta_error(float(atof(prox_string(fp, '\n').c_str())));
	fclose(fp);
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

void led_write(uint8_t num) {
	*led1 = ((num >> 0) & 1);
	*led2 = ((num >> 1) & 1);
	*led3 = ((num >> 2) & 1);
	*led4 = ((num >> 3) & 1);
}

void bat_watcher() {
	double vbat = vin_all_cells->read() * (3.3 * 1470 / 470);
	double threshold = (vbat - 6.6) / 1.4;

	if (threshold >= 0.75) led_write(0b1111);
	else if (threshold >= 0.5) led_write(0b0111);
	else if (threshold >= 0.25) led_write(0b0011);
	else led_write(0b0001);
}

static void receive_cb(const RemoteXBee802 &remote, bool broadcast, const uint8_t *const data, uint16_t len) {
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

int main() {
	led1 = new DigitalOut(LED1);
	led2 = new DigitalOut(LED2);
	led3 = new DigitalOut(LED3);
	led4 = new DigitalOut(LED4);

	vin_all_cells = new AnalogIn(ALL_CELLS);
	vin_single_cell = new AnalogIn(SINGLE_CELL);
	bat_watcher();

	robot = new Robot(messenger);

	log_serial = new Serial(USBTX, USBRX, 115200);

	local = new LocalFileSystem("local");
	ler_arquivo();

	xbee = new XBee802(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 115200);

	#if defined(ENABLE_LOGGING)
	new DigiLoggerMbedSerial(&log_serial, LogLevelInfo);
	#endif

	xbee->register_receive_cb(&receive_cb);

	RadioStatus const radioStatus = xbee->init();
	MBED_ASSERT(radioStatus == XBeeLib::Success);
	xbee->set_network_address(addr);

	Thread t_rx;
	osStatus errTX = t_rx.start(rx_thread); // Handle de erro na thread da serial
	if (errTX) {
		// Tratamento do erro se necessário algum dia
	}
	t_rx.set_priority(osPriorityHigh);

	messenger = new Messenger(robot->MY_ID, robot, xbee);
	robot->start_thread();
	robot->messenger = messenger;

	wait(0.5);
	robot->start_orientation_control(45, 1);
	wait(0.5);
	robot->start_orientation_control(-45, 1);
	wait(0.5);
	robot->start_orientation_control(-45, 1);
	wait(0.5);
	robot->start_orientation_control(45, 1);
	wait(0.5);

	while (true) {
		bat_watcher();
		if (messenger->debug_mode) {
//			Utilizado para eviar dados p/ PC utilizando Messenger
		}
		Thread::wait(1000);
	}
}
