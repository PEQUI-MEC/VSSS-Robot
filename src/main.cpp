#include "mbed.h"
#include "XBeeLib.h"
#include "Messenger.h"
#include "IMU.h"
#include "PIN_MAP.h"
#include "SensorFusion.h"
#include "helper_functions.h"
#include "ConfigFile.h"
#include <cmath>
#include <fstream>
#include <array>
#include <BatteryWatcher.h>

#include <Wheel.h>

#define PI 3.141592f

// void mag_calibration() {
// 	IMU imu{};
// 	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
// 	Thread::wait(1000);

// 	#define sample_size 5000
// 	for (int i = 0; i < sample_size; ++i) {
// 		robot->start_velocity_control(-0.05f, 0.05f);
// 		auto data = imu.read_mag_components();
// 		string msg = std::to_string(data.x) + ',' + std::to_string(data.y);
// 		messenger->send_msg(msg);
// 		Thread::wait(10);
// 	}
// }

// float gyro_calib() {
// 	IMU imu{};
// 	imu.init(IMU_SDA_PIN, IMU_SCL_PIN);
// 	float acc = 0;

// 	#define sample_size_gyro 500
// 	for (int i = 0; i < sample_size_gyro; ++i) {
// 		acc += imu.read_gyro();
// 		wait_ms(5);
// 	}
// 	return acc/sample_size_gyro;
// }

int main() {
	BatteryWatcher battery_watcher;
	battery_watcher.update_battery_leds();

	uint16_t xbee_addr = 0;

	RobotController robot_controller;

	{
		ConfigFile configs("/local/config.txt");
		configs.configure(robot_controller, xbee_addr);
	}

	// xbee->set_complete_callback([](){
	// 	xbee->process_rx_frames();
	// });

	// float offset = gyro_calib();
	float offset = 0;

	SensorFusion sensors;
	sensors.gyro_offset = offset;
	sensors.imu.gyro_scale = robot_controller.gyro_scale;

	Messenger messenger(robot_controller.MY_ID, xbee_addr);

	robot_controller.start_orientation_control(0, 0.8);
	wait(0.1);
	robot_controller.start_orientation_control(-45, 0.8);
	wait(0.5);
	robot_controller.start_orientation_control(0, 0.8);
	wait(0.5);
	robot_controller.start_orientation_control(45, 0.8);
	wait(0.5);
	robot_controller.start_orientation_control(0, 0.8);
	wait(0.5);

	while (true) {

		messenger.process_xbee_msgs();

		messenger.update_by_messages(sensors, robot_controller);

		sensors.update_estimation();

		robot_controller.set_pose(sensors.get_pose());

		robot_controller.control_loop();

		if (robot_controller.stopped) {
			sensors.reset_local_sensors();
		}

		battery_watcher.update_battery_leds();

		messenger.send_info(sensors, robot_controller);

		Thread::wait(10);
	}
}
