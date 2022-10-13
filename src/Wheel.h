#ifndef VSSS_WHEEL_H
#define VSSS_WHEEL_H

#include "PIN_MAP.h"
#include "QEI.h"
#define MOTOR_REVOLUTION_PER_WHEEL_REV 75.8126f

class WheelEncoder {
    QEI encoder;
    Timer timer;
    float velocity = 0;
    float acceleration = 0;

    public:
    WheelEncoder(PinName tach_pin1, PinName tach_pin2) : 
        encoder(tach_pin1, tach_pin2, NC, PULSES_PER_REVOLUTION, QEI::X4_ENCODING) {};

    void update_wheel_velocity();
    void reset();

    float get_velocity() {
        return velocity;
    }
    float get_acceleration() {
        return acceleration;
    }
};

class PID {
	float kp = 1.26;
	float ki = 0.0481;
	float kd = 0;
    float error_acc = 0;
	float last_error = 0;

    public:
    float get_output(float velocity, float target_velocity);
    void reset();
    void set_constants(float kp, float ki, float kd);
};

class WheelController {
    PwmOut pwm_out1;
	PwmOut pwm_out2;
    friend class PID;
    PID pid;

    public:
    WheelController(PinName motor_pin1, PinName motor_pin2)
        : pwm_out1(motor_pin1),
          pwm_out2(motor_pin2) {
        pwm_out1.period_ms(2);
	    pwm_out2.period_ms(2);
    }
    void set_pwm_by_pid(float velocity, float target_velocity);
    void set_pwm(float pwm);
    void stop_and_reset();
    void set_pid_constants(float kp, float ki, float kd);
};

#endif //VSSS_WHEEL_H