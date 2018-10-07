#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "helper_functions.h"
#include "Control.h"
#include "EKF2.h"
#include "EkfModel.h"

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

int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	static Control control;
	static Messenger messenger(&control);

	control.start_threads();
	messenger.start_thread();

//	Serial usb(USBTX, USBRX);

	control.set_target_orientation(to_rads(-45));
	wait(0.5);
	control.set_target_orientation(0);
	wait(0.5);
	control.set_target_orientation(to_rads(45));
	wait(0.5);
	control.set_target_orientation(0);
	wait(0.2);

	control.sensors.timeout.start();

//	control.stop = true;

//	Timer t;
//	t.start();

//	float v = 0;
//	float y = 0;
//	float off = control.sensors.acc_offset;

//	control.set_target_position(0.5, 0.5);

	while (true) {
		if (control.state == ControlState::None) {
			led_write(LEDs, 0);
		} else {
			bat_watcher(LEDs, battery_vin);
		}
		Thread::wait(200);
//		control.set_ang_vel_control(20);
//		Thread::wait(8);

//		auto msg = str(control.sensors.e_time) + "\n";
//		auto acc_y = control.sensors.imu.read_acc().y - off;
//		float time = t.read_us() / 1E6f;
//		t.reset();
//		float v0 = v;
//		v += acc_y * time;
//		y += v0*time + acc_y * std::pow(time,2)/2;

//		auto msg = "$" + str((int) std::round(y * 100)) + ' ' + str((int) std::round(v * 100)) + ";\n";
//		auto msg = str(v) + '\n';
//		auto msg = str(control.sensors.get_pose().theta) + '\n';
//		auto msg = str(control.sensors.last_y_acc) + '\n';
//		auto msg = str(control.sensors.last_controls.ang_accel) + '\n';
//		auto msg = str(control.sensors.last_y_acc) + '\n';
//		auto msg = str(control.sensors.get_pose().w) + '\n';
//		auto msg = str(control.sensors.get_pose().v) + ',' +
//		auto msg = str(control.sensors.last_x_acc) + ',' +
//		messenger.send_log(control.sensors.get_pose().v,
//						   control.sensors.x_acc,
//						   control.sensors.get_pose().w);
//		std::string msg = str(control.sensors.get_pose().v) + ',' +
//				str(control.sensors.x_acc) + ',' +
//				str(control.sensors.x_acc_fixed);
//		std::string msg = str(control.sensors.A) + ',' +
//				str(control.sensors.B);
//				str(control.sensors.previous_w) + '\n';
//		messenger.send_msg(msg);
//		auto msg = str(y) + ',' + str(v) + '\n';
//		auto msg = acc.to_string() + "\n";
//		usb.printf(msg.c_str());
//		usb.printf(msg2.c_str());
	}
}
