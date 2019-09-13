#include <array>
#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "helper_functions.h"
#include "Control.h"
#include "EKF2.h"
#include "EkfModel.h"
#define PI 3.1415926f

Serial usb(USBTX, USBRX, 115200);

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

template <typename ...T>
void send(T ...data) {
	std::string msg;
	append(msg, data...);
	usb.printf("%s\r\n", msg.c_str());
}

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

	control.set_target(ControlState::AngularVel,
					   {0, 0, 0, 20}, false);

//	control.sensors.timeout.start();
	wait(3);

//	control.stop = true;

//	Timer t;
//	t.start();

//	float avg_acc = 0;
//	float avg_rsin_acc = 0;
//	float avg_rcos_acc = 0;
//	float offset_acc = 0;
//	int count = 0;
//	float ax_acc = 0, ay_acc = 0, az_acc = 0;
//	float y = 0;
//	float off = control.sensors.acc_offset;

	auto& s = control.sensors;

	while (true) {
		if (control.state == ControlState::None) {
			led_write(LEDs, 0);
		} else {
			bat_watcher(LEDs, battery_vin);
		}
		Thread::wait(100);

//		messenger.send_log(control.sensors.btime);

//		auto msg = std::to_string(control.sensors.btime);
//		usb.printf("%s\r\n", msg.c_str());
//		usb.printf("%f, %f, %f\r\n", control.sensors.gravity, control.sensors.theta_x,
//				   control.sensors.theta_y);
//		usb.printf("%f, %f, %f\r\n", control.sensors.theta_x, control.sensors.theta_y,
//				   control.sensors.theta_z);
//		send(control.sensors.theta_x, control.sensors.theta_y, control.sensors.theta_z);
		send(deg(control.sensors.theta_x), deg(control.sensors.theta_y), deg(control.sensors.theta_z));
//		send(deg(control.sensors.theta_x), deg(control.sensors.theta_y), control.sensors.gravity);
//		send(s.ctrl.gyro(0), s.ctrl.gyro(1), s.ctrl.gyro(2), deg(s.theta_x), deg(s.theta_y), deg(s.theta_z));
//		send(s.ctrl.gyro(0), s.ctrl.gyro(1), s.ctrl.gyro(2));
//		usb.printf("%s\r\n", control.sensors.ctrl.to_str().c_str());
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
//		messenger.send_log(control.sensors.theta_x);
//		messenger.send_log(control.sensors.get_pose().theta_y);
//		auto acc = control.sensors.acc_test;
//		float ag = std::sqrt(std::pow(acc.y, 2.0f)
//							 + std::pow(acc.z, 2.0f));
//		float theta = std::atan2(-acc.z, acc.y) + PI/2;
//		float theta = control.sensors.get_pose().theta_y;
//		float ag = control.sensors.gravity;
//		float ar = (acc.y - ag * std::sin(theta)) / std::cos(theta);
//		float elapsed = t.read();
//		t.reset();
//		float v0 = v;
//		v += ar * elapsed;
//		auto pose = control.sensors.get_pose();
//		messenger.send_log(pose.x, pose.y, pose.v, pose.theta_y);
//		auto acc = control.sensors.acc_test;
//		if (count == 1000) {
//			avg_rsin_acc = 0;
//			avg_rcos_acc = 0;
//			ax_acc = 0;
//			ay_acc = 0;
//			az_acc = 0;
//			offset_acc = 0;
//			avg_acc = 0;
//			count = 0;
//		}
//		auto acc = control.sensors.acc_real;
//		avg_acc += acc.x;
//		ax_acc += acc.x;
//		ay_acc += acc.y;
//		az_acc += acc.z;
//		avg_rsin_acc += control.sensors.r_sin;
//		avg_rcos_acc += control.sensors.r_cos;
//		offset_acc += control.sensors.gyro_yx_m;
//		count++;
//		messenger.send_log(offset_acc / count,
//						   control.sensors.gyro_yx_m);

//		messenger.send_log(control.sensors.get_pose().theta,
//						   control.sensors.get_pose().w,
//						   control.sensors.alpha);
//		auto mod = std::sqrt(std::pow(ax_acc / count, 2.0f)
//							+ std::pow(ay_acc / count, 2.0f) + std::pow(az_acc / count, 2.0f));
//		messenger.send_log(ax_acc / count, ay_acc / count, az_acc / count, mod);
//		messenger.send_log(acc.x, acc.y);
//		messenger.send_log(avg_rsin_acc / count,
//						   control.sensors.r_sin,
//						   avg_rcos_acc / count,
//						   control.sensors.r_cos,
//						   control.sensors.get_pose().theta_y);
//						   control.sensors.ax, control.sensors.ay,
//						   control.sensors.ax_raw,
//						   control.sensors.get_pose().theta_y);
//						   offset_acc / count, offset);
//		messenger.send_log(avg_acc/count, acc.x, acc.y, acc.z);
//		messenger.send_log(v, ar, ag, theta);
//		auto acc = control.sensors.acc_real;
//		float ag = control.sensors.gravity;
//		auto ar = std::sqrt(std::pow(acc.x, 2.0f)
//							+ std::pow(acc.z, 2.0f) - std::pow(ag, 2.0f));
//		auto theta = 2 * (std::atan( (ar - acc.x) / (acc.z + ag) ) + 0 * PI);
//		messenger.send_log(acc.x,
//						   acc.y,
//						   acc.z,
//		messenger.send_log(control.sensors.ay,
//						   control.sensors.ax,
//						   control.sensors.get_pose().theta_y);
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
