#include "QEI.h"
#include "PIN_MAP.h"

struct wheel {
	QEI* encoder;
	PwmOut* pwm_out1;
	PwmOut* pwm_out2;
	float velocity;
	float target_velocity;
	float error_acc;
	float last_error;
};

void init_wheel(wheel& w, PinName tach_pin1, PinName tach_pin2, PinName motor_pin1, PinName motor_pin2) {
	w.encoder = new QEI(tach_pin1, tach_pin2, NC, PULSES_PER_REVOLUTION, QEI::X4_ENCODING);
	w.pwm_out1 = new PwmOut(motor_pin1);
	w.pwm_out1->period_ms(2);
	w.pwm_out2 = new PwmOut(motor_pin2);
	w.pwm_out2->period_ms(2);
	w.error_acc = 0;
}

void set_pwm(wheel &w, float pwm) {
	if(pwm > 1) pwm = 1;
	if(pwm < -1) pwm = -1;
	if(std::abs(pwm) < 0.05) pwm = 0;
//	pwm = 0;

	if (pwm < 0) {
		w.pwm_out1->write(1);
		w.pwm_out2->write(1 + pwm);
	} else if (pwm > 0) {
		w.pwm_out1->write(1 - pwm);
		w.pwm_out2->write(1);
	} else {
		w.pwm_out1->write(1);
		w.pwm_out2->write(1);
	}
}


int main() {
	wheel left_wheel = {};
	wheel right_wheel = {};
	init_wheel(left_wheel, ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2, MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2);
	init_wheel(right_wheel, ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2);

	set_pwm(left_wheel, 1);
	set_pwm(right_wheel, 1);
}