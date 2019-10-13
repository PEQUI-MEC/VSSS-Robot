#include "EkfModel.h"
#include "helper_functions.h"

using EKF = EkfModel::EKF;

EKF::PoseVec EkfModel::prediction(const EKF::PoseVec &prev_x, const Controls &controls,
		const WheelVelocity &wheel_vel, float time) {
	Pose pose(prev_x);
	Pose pred;

	auto vel = (wheel_vel.left + wheel_vel.right)/2;
	auto ang_acc_z = controls.gyro(2) - last_gyro_z; //sem div por tempo
	auto vel_acc_x = vel - last_vel_x; //sem div por tempo
	last_gyro_z = controls.gyro(2);
	last_vel_x = vel;

	float delta_theta = (pose.w * time) / 2;
	float x_direction = time * std::cos(pose.theta + delta_theta);
	float x_increment = pose.v * x_direction;
	float y_direction = time * std::sin(pose.theta + delta_theta);
	float y_increment = pose.v * y_direction;

	pred.x = pose.x + x_increment;
	pred.y = pose.y + y_increment;

	pred.theta = wrap(pose.theta + controls.gyro(2)* time);

	pred.v = pose.v + vel_acc_x;
	pred.w = pose.w + ang_acc_z;

	F(0, 2) = -y_increment;
	F(1, 2) = x_increment;
	F(0, 3) = x_direction;
	F(1, 3) = y_direction;
	F(0, 4) = -y_increment * time / 2;
	F(1, 4) = x_increment * time / 2;

	process_noise(time);
	return pred.to_vec();
}

void EkfModel::process_noise(float time) {
	R(0, 0) = time * 0.001f;
	R(1, 1) = time * 0.001f;
	R(2, 2) = time * 0.00001f;
	R(3, 3) = time * 0.0001f;
	R(4, 4) = time * 0.0001f;
}

EKF::SensorVec EkfModel::sensor_measurement_error(const EKF::PoseVec &x, const EKF::SensorVec &z) {
	auto pred_z = sensor_measurement_model(x);
	EKF::SensorVec z_error = z - pred_z;
	z_error(0,0) = wrap(z_error(0,0));
	return z_error;
}

EKF::SensorVec EkfModel::sensor_measurement_model(const EKF::PoseVec &x) {
	EKF::SensorVec z;
	z(0,0) = x(2,0);
	z(1,0) = x(4,0);
	float v_increment = x(4,0) * ROBOT_SIZE/2;
	z(2,0) = x(3,0) - v_increment;
	z(3,0) = x(3,0) + v_increment;
	return z;
}

EKF::VisionVec EkfModel::vision_measurement_error(const EKF::PoseVec &x, const EKF::VisionVec &z) {
	auto pred_z = vision_measurement_model(x);
	EKF::VisionVec error = z - pred_z;
	error(2,0) = wrap(error(2,0));
	return error;
}

EKF::VisionVec EkfModel::vision_measurement_model(const EKF::PoseVec &x) {
	EKF::VisionVec z;
	for (int i = 0; i < 3; ++i) {
		z(i,0) = x(i,0);
	}
	return z;
}


void EkfModel::use_magnetometer(bool use) {
	if(use) H(0,2) = 1;
	else H(0,2) = 0;
}

void EkfModel::use_encoders(bool use) {
	if(use) {
		H(2,3) = 1;
		H(3,3) = 1;
		H(2,4) = -ROBOT_SIZE / 2;
		H(3,4) = ROBOT_SIZE / 2;
	} else {
		H(2,3) = 0;
		H(3,3) = 0;
		H(2,4) = 0;
		H(3,4) = 0;
	}
}

EkfModel::EkfModel() {
	F.setIdentity();
	R.setZero();

	H.setZero();
	H(0,2) = 1;
	H(1,4) = 1;
	H(2,3) = 1;
	H(3,3) = 1;
	H(2,4) = -ROBOT_SIZE/2;
	H(3,4) = ROBOT_SIZE/2;

	Q.setZero();
	Q(0,0) = 0.00022846f;
	Q(1,1) = 0.02857541f;
	Q(2,2) = 0.00022096f;
	Q(3,3) = 0.00022096f;

	Hv.setZero();
	for (int i = 0; i < 3; ++i) {
		Hv(i,i) = 1;
	}

	Qv.setZero();
	Qv(0,0) = 3.44048681e-06f;
	Qv(1,1) = 2.82211659e-06f;
	Qv(2,2) = 9.75675349e-04f;
}
