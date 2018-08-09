#ifndef VSSS_VFOCONTROL_H
#define VSSS_VFOCONTROL_H

#include <cmath>
#include <tuple>
#include "SensorFusion.h"
#include "Controller.h"

struct vec {
	float x;
	float y;

	inline float vec_norm() {
		return std::sqrt(x*x + y*y);
	}

	inline vec operator+(vec v2){
		return {x + v2.x, y + v2.y};
	}

	inline vec operator-(vec v2){
		return {x - v2.x, y - v2.y};
	}

	inline vec operator*(float num) {
		return {num * x, num * y};
	}

	inline float operator*(vec v2) {
		return x * v2.x + y*v2.y;
	}
};

struct pose_vector {
	vec position;
	float theta;
	float v;
};

struct target_data {
	float x;
	float y;
	float theta;
};

class VFOControl {
	public:
		float k1 = 4;
		float kp = 1;
		float n = 0.8;
		float sgnk = 1;

		float previous_sigma = 0;
		bool first_sigma = true;
		float previous_fi_a = 0;
		bool first_fi_a = true;

		target_data target{};

		SensorFusion* sensors;
		Controller controller;
		Thread vfo_thread;

		VFOControl();
		void vfo_control_loop();
		void start_thread();
		vec get_controls(vec target_p, float target_theta, pose_vector &pose);
		std::tuple<vec, vec> get_vector_field(vec target_p, float target_theta, pose_vector &pose);
		vec get_hp_dot(vec &target_g2, float sigma, pose_vector &pose);
		float get_sigma(vec error);
		void set_sgnk(float target_x, float x);
		vec get_g2(float theta);
		float atan2c(float g3, float g2);
		float F(float angle);
};

#endif //VSSS_VFOCONTROL_H
