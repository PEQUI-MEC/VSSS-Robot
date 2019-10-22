#include <array>
#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "helper_functions.h"
#include "Control.h"
#include "EKF2.h"
#include "EkfModel.h"
#define PI 3.1415926f

//Serial usb(USBTX, USBRX, 115200);

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

float rd(float value) {
	return std::round(value * 100) / 100;
}

//template <typename ...T>
//void send(T ...data) {
//	std::string msg;
//	append(msg, data...);
//	usb.printf("%s\r\n", msg.c_str());
//}

float deg(float rad) {
	return rd(rad * (180.0f/PI));
}

int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	Control control;
	Messenger messenger(&control);

	control.start_threads();

//	Serial usb(USBTX, USBRX);

	auto to_orientation = [&](float degrees) {
		control.set_target(ControlState::Orientation,
						   {0, 0, to_rads(degrees), 0}, true);
		wait(0.5);
	};

	to_orientation(-45);
	to_orientation(0);
	to_orientation(45);
	to_orientation(0);

	messenger.start_thread();

//	control.sensors.timeout.start();

//	control.set_target(ControlState::Vector,
//					   {0, 0, to_rads(90+45), 1.4f}, true);

	auto& s = control.sensors;
	auto& c = *s.controller;

	Timer t;
	t.start();
	constexpr int time = 1000 * 60 * 2;
	constexpr float max = 36;

	while (true) {
		if (t.read_ms() == max) {
			control.set_target(ControlState::None,
							   {0, 0, 0, 0}, false);
		} else {
			float vel = (max * t.read_ms()) / time;
			control.set_target(ControlState::AngularVel,
							   {0, 0, 0, vel}, false);
			messenger.send_log(s.imu_data.gyro(2), c.left_wheel.velocity, c.right_wheel.velocity);
		}

		if (control.state == ControlState::None) {
			led_write(LEDs, 0);
		} else {
			bat_watcher(LEDs, battery_vin);
		}
		Thread::wait(10);
//		EKF2::T::VisionVec & error = control.sensors.ekf.model.vision_error;
//		messenger.send_log(error(0), error(1), error(2));
//		messenger.send_log(deg(s.theta_x), deg(s.theta_y), deg(s.theta_z));
	}
}
