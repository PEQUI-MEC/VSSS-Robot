//
// Created by thiago on 12/07/18.
//

#ifndef VSSS_HELPER_FUNCTIONS_H
#define VSSS_HELPER_FUNCTIONS_H

#include <string>
#include <cmath>

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

#endif //VSSS_HELPER_FUNCTIONS_H
