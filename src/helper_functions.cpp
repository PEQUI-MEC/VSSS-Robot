#include "helper_functions.h"

float to_rads(float degrees) {
	return degrees * 3.1415926f/180.0f;
}

float wrap(float angle) {
	static constexpr float PI = 3.1415926f;
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}

Vector operator*(float value, Vector vec) {
	return {value * vec.x, value * vec.y };
}
