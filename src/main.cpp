#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
//#include "SensorFusion.h"
#include "helper_functions.h"
//#include "ConfigFile.h"
#include "Control.h"
//#include "VFO.h"
#include <cmath>
#include <fstream>
#include <AdpsSensor.h>

#define PI 3.141592f

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

TargetPose next_target(pose_data pose) {
//	float theta = PI/4 + pose.theta;
	float theta = 0;
	float distance = 0.04;
	float x = pose.x + distance*std::cos(theta);
	float y = pose.y + distance*std::sin(theta);
	return {x, y, theta};
}

int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	static Control control;
//	static VFO control;
	static Messenger messenger('B', &control);
//	static Serial usb(USBTX, USBRX);
//	static AdpsSensor adps(p9, p10);
	static AdpsSensor adps(IMU_SDA_PIN, IMU_SCL_PIN);

	control.start_threads();
	messenger.start_thread();

	control.set_target_pose(0, 0, to_rads(-45));
	wait(0.5);
	control.set_target_pose(0, 0, 0);
	wait(0.5);
	control.set_target_pose(0, 0, to_rads(45));
	wait(0.5);
	control.set_target_pose(0, 0, 0);
	wait(0.2);
//	control.velocity = 0.8;
//	control.set_target_pose(0.5, 0, PI);

//	control.set_target_pose(0.5, 0.5, -PI/2);

//	Thread::wait(2000);
//	control.set_target_pose(0.5,0.5,0);
//
//	Thread::wait(2000);
//	control.set_target_pose(0,0,PI);

	while (true) {
//		auto pose = control.sensors.get_pose();
//		double vel = control.sensors.prev_mesure.vel_left;
//		std::string msg = str(control.sensors.get_pose().w) + '\n';
//		usb.printf(msg.c_str());
//		uint8_t data = adps.read_proximity();
//		std::string msg = "read: " + std::to_string(data) + "\n";
//		usb.printf(msg.c_str());
//		auto color = adps.read_color();
//		usb.printf(color.to_string().c_str());
//
//		static constexpr float thresh = 1.2;
//		bool is_ball = float(color.r)/color.b > thresh && float(color.r)/color.g > thresh;
//		if(is_ball) usb.printf("is ball\n");
//		else usb.printf("isn't ball\n");
		auto location = adps.read_location();
		auto theta = float(std::atan2(location.right, location.left)) - to_rads(45);
		auto prox = float(std::sqrt(std::pow(location.right, 2.0f) + std::pow(location.left, 2.0f)));
		if(prox > 10) {
			control.set_target_pose(0, 0, wrap(control.sensors.get_pose().theta - theta));
		} else {
			control.set_target_pose(0, 0, control.sensors.get_pose().theta);
		}
//		usb.printf(location.to_string().c_str());
//		auto msg = str(std::atan2(location.right, location.left)) + '\n';
//		usb.printf(msg.c_str());
//		auto next = next_target(control.sensors.get_pose());
//		control.set_target_pose(next.x, next.y, next.theta);
		bat_watcher(LEDs, battery_vin);
//		Thread::wait(10);
		Thread::wait(200);
	}
}
