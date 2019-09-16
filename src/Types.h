#ifndef VSSS_TYPES_H
#define VSSS_TYPES_H

#include <Core>
#include "helper_functions.h"

struct Pose {
	float x = 0;
	float y = 0;
	float theta = 0;
	float v = 0;
	float w = 0;
	float theta_y = 0;

	static constexpr uint32_t SIZE = 6;

	Pose() = default;

	explicit Pose(const Eigen::Matrix<float, SIZE, 1> &pose_vec) :
			x(pose_vec(0, 0)), y(pose_vec(1, 0)),
			theta(pose_vec(2, 0)), v(pose_vec(3, 0)),
			w(pose_vec(4, 0)), theta_y(pose_vec(5, 0)) {}

	Eigen::Matrix<float, SIZE, 1> to_vec() {
		Eigen::Matrix<float, SIZE, 1> vec;
		vec(0, 0) = x;
		vec(1, 0) = y;
		vec(2, 0) = theta;
		vec(3, 0) = v;
		vec(4, 0) = w;
		vec(5, 0) = theta_y;
		return vec;
	}

//	Pose or_backwards(bool backwards) {
//		static constexpr float PI = 3.1415926f;
//		if (!backwards) return *this;
//		else return {x, y, wrap(theta + PI), -v, w, theta_y, w_y};
//	}
};

struct Controls {
	using Vec3 = Eigen::Vector3f;
	Vec3 gyro;
	Vec3 acc;

	static constexpr uint32_t SIZE = 6;

	std::string to_csv() {
		return str(gyro(0)) + ", " + str(gyro(1)) +
			   ", " + str(gyro(2)) + ", " +
			   str(acc(0)) + ", " + str(acc(1)) +
			   ", " + str(acc(2));

	}

	std::string to_str() {
		return "[ " + str(gyro(0)) + ", " + str(gyro(1)) +
				", " + str(gyro(2)) + "], [ " +
				str(acc(0)) + ", " + str(acc(1)) +
				", " + str(acc(2)) + " ]";

	}
};

//struct Controls {
//	float lin_accel;
//	float ang_accel;
//	float gyro_y;
//
//	explicit Controls(Eigen::Matrix<float, 3, 1> control_vec) :
//			lin_accel(control_vec(0, 0)),
//			ang_accel(control_vec(1, 0)),
//			gyro_y(control_vec(2, 0)) {}
//
//	Controls(float lin_accel, float ang_accel, float gyro_y) :
//			lin_accel(lin_accel), ang_accel(ang_accel), gyro_y(gyro_y) {}
//
//	Eigen::Matrix<float, 3, 1> to_vec() {
//		Eigen::Matrix<float, 3, 1> vec;
//		vec(0, 0) = lin_accel;
//		vec(1, 0) = ang_accel;
//		vec(2, 0) = gyro_y;
//		return vec;
//	}
//};

struct SensorData {
	float mag_theta = 0;
	float gyro;
	float vel_left = 0;
	float vel_right = 0;

	static constexpr uint32_t SIZE = 4;

	SensorData() = default;

	SensorData(float mag_theta, float gyro, float vel_left, float vel_right) :
			mag_theta(mag_theta), gyro(gyro),
			vel_left(vel_left), vel_right(vel_right) {}

	Eigen::Matrix<float, SIZE, 1> to_vec() {
		Eigen::Matrix<float, SIZE, 1> vec;
		vec(0, 0) = mag_theta;
		vec(1, 0) = gyro;
		vec(2, 0) = vel_left;
		vec(3, 0) = vel_right;
		return vec;
	}
};

struct VisionData {
	float x = 0;
	float y = 0;
	float theta = 0;

	static constexpr uint32_t SIZE = 3;

	VisionData() = default;

	VisionData(float x, float y, float theta) :
			x(x), y(y), theta(theta) {}

	Eigen::Matrix<float, 3, 1> to_vec() {
		Eigen::Matrix<float, 3, 1> vec;
		vec(0, 0) = x;
		vec(1, 0) = y;
		vec(2, 0) = theta;
		return vec;
	}
};

template<int POSE_SIZE, int SENSOR_SIZE, int VISION_SIZE, int CONTROL_SIZE>
class EKFTypes {
	public:
	template<int x, int y> using Matrix = Eigen::Matrix<float, x, y>;
	using PoseVec = Matrix<POSE_SIZE, 1>;
	using PoseMat = Matrix<POSE_SIZE, POSE_SIZE>;
	using ControlVec = Matrix<CONTROL_SIZE, 1>;

	using SensorVec = Matrix<SENSOR_SIZE, 1>;
	using SensorMat = Matrix<SENSOR_SIZE, SENSOR_SIZE>;
	using HSensorMat = Matrix<SENSOR_SIZE, POSE_SIZE>;
	using KSensorMat = Matrix<POSE_SIZE, SENSOR_SIZE>;

	using VisionVec = Matrix<VISION_SIZE, 1>;
	using VisionMat = Matrix<VISION_SIZE, VISION_SIZE>;
	using HVisionMat = Matrix<VISION_SIZE, POSE_SIZE>;
	using KVisionMat = Matrix<POSE_SIZE, VISION_SIZE>;
};

#endif //VSSS_TYPES_H
