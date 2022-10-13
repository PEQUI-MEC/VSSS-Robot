//
// Created by thiago on 24/04/18.
//

#ifndef VSSS_EKF_H
#define VSSS_EKF_H

#include <Sparse>
#include <Eigen>
#include <Core>

#include "QEI.h"
#include "IMU.h"
#include "PIN_MAP.h"

#define POSE_SIZE 5
#define MEASUREMENT_SIZE 4
#define MEASUREMENT_SIZE_CAM 3
#define ROBOT_SIZE 0.0675f

struct pose_data {
	float x;
	float y;
	float theta;
	float v;
	float w;

	float left_wheel_vel;
	float right_wheel_vel;
};

struct measurement_data {
	float mag_theta;
	float gyro_w;
	float vel_left;
	float vel_right;
};

struct vision_data {
	float x;
	float y;
	float theta;
};

class EKF {
	public:
		pose_data pose{};

		Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> F;
		Eigen::Matrix<float, POSE_SIZE, 1> x;
		Eigen::Matrix<float, POSE_SIZE, 1> x_p;
		Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> COV_P;
		Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> COV;
		Eigen::Matrix<float, MEASUREMENT_SIZE, MEASUREMENT_SIZE> Q;
		Eigen::Matrix<float, MEASUREMENT_SIZE, 1> z;
		Eigen::Matrix<float, MEASUREMENT_SIZE, POSE_SIZE> H;

		Eigen::Matrix<float, MEASUREMENT_SIZE_CAM, POSE_SIZE> H_CAM;
		Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> I;

		Eigen::Matrix<float, MEASUREMENT_SIZE_CAM, MEASUREMENT_SIZE_CAM> Q_CAM;
		Eigen::Matrix<float, MEASUREMENT_SIZE_CAM, 1> z_cam;

		Eigen::Matrix<float,MEASUREMENT_SIZE_CAM,1> camera_measurement_model();

		void predict(float time, float left_accel, float right_accel, float ang_accel);
		void update(measurement_data data, bool use_mag, bool use_enc);
		void update_camera(vision_data data);
		Eigen::Matrix<float,MEASUREMENT_SIZE,1> measurement_model();
		float round_angle(float angle);
		EKF();
		Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> process_noise(float time);
};

#endif //VSSS_EKF_H
