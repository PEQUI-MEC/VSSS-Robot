//
// Created by thiago on 24/04/18.
//

#include "EKF.h"
#include <tuple>
#include <cmath>

#define PI 3.141592f

void EKF::predict(float time, float left_accel, float right_accel, float ang_accel) {
	float linear_accel = (right_accel + left_accel)/2;
//	float angular_accel = (right_accel - left_accel)/ROBOT_SIZE;

//	Predicts pose
	float delta_theta = (pose.w * time)/2;

	float x_direction = time * std::cos(pose.theta + delta_theta);
	float x_increment = pose.v * x_direction;

	float y_direction = time * std::sin(pose.theta + delta_theta);
	float y_increment = pose.v * y_direction;

	x_p(0,0) = pose.x + x_increment;
	x_p(1,0) = pose.y + y_increment;
	x_p(2,0) = round_angle(pose.theta + pose.w * time);
	x_p(3,0) = pose.v + linear_accel;
	x_p(4,0) = pose.w + ang_accel;

//	Computes jacobian
	F(0,2) = -y_increment;
	F(1,2) = x_increment;
	F(0,3) = x_direction;
	F(1,3) = y_direction;
	F(0,4) = -y_increment * time/2;
	F(1,4) = x_increment * time/2;

//	Predicted covariance
	auto R = process_noise(time);
	COV_P = F * COV * F.transpose() + R;
}

void EKF::update(measurement_data data, bool use_mag, bool use_enc) {
//	Sets z
	z(0,0) = data.mag_theta;
	z(1,0) = data.gyro_w;
	z(2,0) = data.vel_left;
	z(3,0) = data.vel_right;

//	Sets magnetometer use
	if(use_mag) H(0,2) = 1;
	else H(0,2) = 0;

	if(use_enc) {
		H(2,3) = 1;
		H(3,3) = 1;
		H(2,4) = -ROBOT_SIZE/2;
		H(3,4) = ROBOT_SIZE/2;
	} else {
		H(2,3) = 0;
		H(3,3) = 0;
		H(2,4) = 0;
		H(3,4) = 0;
	}

//	Kalman Gain
	Eigen::Matrix<float, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = H * COV_P * H.transpose() + Q;
	Eigen::Matrix<float, POSE_SIZE, MEASUREMENT_SIZE> K_GAIN = COV_P * H.transpose() * S.inverse();

//	Updated Pose
	auto pred_z = measurement_model();
	Eigen::Matrix<float, MEASUREMENT_SIZE, 1> error = z - pred_z;
	error(0,0) = round_angle(error(0,0));
	x = x_p + K_GAIN * error;

//	std::tie(pose.x, pose.y, pose.theta, pose.v, pose.w) = std::make_tuple(x(0,0), x(1,0), x(2,0), x(3,0), x(4,0));
	pose.x = x(0,0);
	pose.y = x(1,0);
	pose.theta = x(2,0);
	pose.v = x(3,0);
	pose.w = x(4,0);

//	Updated Covariance
	COV = (I - K_GAIN * H) * COV_P;
}

void EKF::update_camera(vision_data data) {
//	Sets z
	z_cam(0,0) = data.x;
	z_cam(1,0) = data.y;
	z_cam(2,0) = data.theta;

//	Kalman Gain
	Eigen::Matrix<float, MEASUREMENT_SIZE_CAM, MEASUREMENT_SIZE_CAM> S = H_CAM * COV_P * H_CAM.transpose() + Q_CAM;
	Eigen::Matrix<float, POSE_SIZE, MEASUREMENT_SIZE_CAM> K_GAIN = COV_P * H_CAM.transpose() * S.inverse();

//	Updated Pose
	auto pred_z = camera_measurement_model();
	Eigen::Matrix<float, MEASUREMENT_SIZE_CAM, 1> error = z_cam - pred_z;
	error(2,0) = round_angle(error(2,0));
	x = x_p + K_GAIN * error;

//	std::tie(pose.x, pose.y, pose.theta, pose.v, pose.w) = std::make_tuple(x(0,0), x(1,0), x(2,0), x(3,0), x(4,0));
	pose.x = x(0,0);
	pose.y = x(1,0);
	pose.theta = x(2,0);
	pose.v = x(3,0);
	pose.w = x(4,0);

//	Updated Covariance
	COV = (I - K_GAIN * H_CAM) * COV_P;
}

Eigen::Matrix<float,MEASUREMENT_SIZE,1> EKF::measurement_model() {
	Eigen::Matrix<float,MEASUREMENT_SIZE,1> pred_measurement;
	pred_measurement(0,0) = x_p(2,0);
	pred_measurement(1,0) = x_p(4,0);
	float v_increment = x_p(4,0) * ROBOT_SIZE/2;
	pred_measurement(2,0) = x_p(3,0) - v_increment;
	pred_measurement(3,0) = x_p(3,0) + v_increment;
	return pred_measurement;
}

Eigen::Matrix<float,MEASUREMENT_SIZE_CAM,1> EKF::camera_measurement_model() {
	Eigen::Matrix<float,MEASUREMENT_SIZE_CAM,1> pred_measurement;
	for (int i = 0; i < 3; ++i) {
		pred_measurement(i,0) = x_p(i,0);
	}
	return pred_measurement;
}

Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> EKF::process_noise(float time) {
	Eigen::Matrix<float, POSE_SIZE, POSE_SIZE> R;
	R.setZero();
	R(0,0) = time * 0.0001f;
	R(1,1) = time * 0.0001f;
	R(2,2) = time * 0.00001f;
	R(3,3) = time * 0.0001f;
	R(4,4) = time * 0.0001f;
	return R;
};

float EKF::round_angle(float angle) {
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}

EKF::EKF() {
//	Measurement model for gyroscope, magnetometer and encoders
	H.setZero();
	H(0,2) = 1;
	H(1,4) = 1;
	H(2,3) = 1;
	H(3,3) = 1;
	H(2,4) = -ROBOT_SIZE/2;
	H(3,4) = ROBOT_SIZE/2;

//	Measurement for vision (x, y and theta)
	H_CAM.setZero();
	for (int i = 0; i < 3; ++i) {
		H_CAM(i,i) = 1;
	}

	F.setIdentity();
	x_p.setZero();
	z.setConstant(1.2);
	z_cam.setConstant(1.2);
	I.setIdentity();
	COV.setIdentity();
	COV_P.setZero();

	Q.setZero();
	Q(0,0) = 0.00022846f;
	Q(1,1) = 0.002857541f;
	Q(2,2) = 0.00022096f;
	Q(3,3) = 0.00022096f;

	Q_CAM.setZero();
	Q_CAM(0,0) = 3.44048681e-06f;
	Q_CAM(1,1) = 2.82211659e-06f;
	Q_CAM(2,2) = 9.75675349e-04f;
}
