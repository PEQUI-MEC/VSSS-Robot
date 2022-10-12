#ifndef VSSS_WHEEL_H
#define VSSS_WHEEL_H

#include "PIN_MAP.h"
#include "QEI.h"
#define MOTOR_REVOLUTION_PER_WHEEL_REV 75.8126f

class WheelEncoder {
    QEI encoder;
    float velocity;
    WheelEncoder(PinName tach_pin1, PinName tach_pin2) : 
        encoder(tach_pin1, tach_pin2, NC, PULSES_PER_REVOLUTION, QEI::X4_ENCODING) {};
};

struct PID {
	float kp;
	float ki;
	float kd;
    float error_acc;
	float last_error;
};

class WheelController {
    PwmOut* pwm_out1;
	PwmOut* pwm_out2;
    PID pid;
    float target_velocity;
};

#endif //VSSS_WHEEL_H