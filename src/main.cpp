#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "helper_functions.h"
#include "Control.h"
//#include "EKF2.h"
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

void print(Messenger &msg, const UKF::T::UKFSigmaMat &sig) {
	for (int i = 0; i < 6; i++) {
		std::string m;
		for (int j = 0; j < 13; ++j) {
			m.append(str(sig(i, j)));
			m.append(",");
		}
		msg.send_msg(m);
	}
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

	auto to_orientation = [&](float degrees) {
		control.set_target(ControlState::Orientation,
						   {0, 0, to_rads(degrees), 0}, true);
		wait(0.5);
	};

//	to_orientation(-45);
//	to_orientation(0);
//	to_orientation(45);
//	to_orientation(0);

//	control.set_target(ControlState::Position,
//					   {0.5, 0.5, 0, 0.8f}, true);


//	control.set_target(ControlState::Position,
//					   {0.5, 0.5, 0, 0.8f}, true);

	control.set_target(ControlState::WheelVel,
					   {0.5, 0.6, 0, 0.8f}, true);
//	wait(2);
//	control.sensors.timeout.start();

//	control.stop = true;

//	Timer t;
//	t.start();

//	float v = 0;
//	float y = 0;
//	float off = control.sensors.acc_offset;

	while (true) {
		if (control.state == ControlState::None) {
			led_write(LEDs, 0);
		} else {
			bat_watcher(LEDs, battery_vin);
		}
		Thread::wait(10);
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
//		auto pose = control.sensors.get_pose();
//		auto& cov = control.sensors.ukf.COV;
//		auto& u = control.sensors.ukf;
//		if (u.new_log) {
//			messenger.send_log(u.x_error, u.y_error, u.theta_error);
//			u.new_log = false;
//		}
		print(messenger, control.sensors.ukf.X);
		messenger.send_log(0);
//		messenger.send_log(pose.x, pose.y, pose.theta,
//						   cov(0,0), cov(1,1), cov(2,2));
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
