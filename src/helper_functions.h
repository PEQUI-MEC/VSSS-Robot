//
// Created by thiago on 12/07/18.
//

#ifndef VSSS_HELPER_FUNCTIONS_H
#define VSSS_HELPER_FUNCTIONS_H

#include <string>
#include <cmath>

inline float min(float a, float b) {
	return (a < b) ? a : b;
}

inline float limit_error(float target, float current, float max_err) {
	float diff = target - current;
	if (std::abs(diff) > max_err) {
		if (target > current) {
			return current + max_err;
		} else {
			return current - max_err;
		}
	} else {
		return target;
	}
}

template <typename T>
inline std::string str(const T& x) {
	return std::to_string(x);
}

float to_rads(float degrees);

float wrap(float angle);

template <typename T>
struct WithDerivative {
	T value;
	T derivative;
};

struct Vector {
	float x;
	float y;

	explicit Vector(float orientation) :
			x(std::cos(orientation)),
			y(std::sin(orientation)) {}
	Vector(float x, float y) : x(x), y(y) {}
	float modulus() const {
		return std::sqrt(std::pow(x, 2.0f) + std::pow(y, 2.0f));
	}
	float operator*(const Vector vec) const { return x * vec.x + y * vec.y; }
	Vector operator+(const Vector vec) const { return {x + vec.x, y + vec.y}; }
};

struct Point {
	float x;
	float y;
	Vector operator-(const Point p) const { return {x - p.x, y - p.y}; }
};

Vector operator*(float value, Vector vec);

// Appends csv data
template <typename T>
void append(std::string &buff, T last) {
	buff.append(std::to_string(last));
//	buff += '\n';
}

template <typename First, typename ...Others>
void append(std::string &buff,
			First first, Others ...others) {
	append(buff, first);
	buff += ',';
	append(buff, others...);
}

#endif //VSSS_HELPER_FUNCTIONS_H
