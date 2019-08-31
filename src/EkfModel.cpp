#include "EkfModel.h"
#include "helper_functions.h"

using EKF = EkfModel::EKF;

EKF::PoseVec EkfModel::prediction(const EKF::PoseVec &prev_x,
								  const Controls &controls, float time) {
	Pose pose(prev_x);
	Pose pred;
	update_rot_mats(0, pose.theta_y, pose.theta);

	float delta_theta = (pose.w * time) / 2;
	float x_direction = time * std::cos(pose.theta + delta_theta);
	float x_increment = pose.v * x_direction;
	float y_direction = time * std::sin(pose.theta + delta_theta);
	float y_increment = pose.v * y_direction;

	auto acc = apply_rotation(controls.acc);

	pred.x = pose.x + x_increment;
	pred.y = pose.y + y_increment;

	pred.theta = wrap(pose.theta + pose.w * time);
	pred.theta_y = wrap(pose.theta_y + pose.w_y * time);

	pred.v = pose.v + acc(0, 0) * time;
	pred.w = pose.w + ang_acc_z * time;
	pred.w_y = pose.w_y + ang_acc_y * time;

	F(0, 2) = -y_increment;
	F(1, 2) = x_increment;
	F(0, 3) = x_direction;
	F(1, 3) = y_direction;
	F(0, 4) = -y_increment * time / 2;
	F(1, 4) = x_increment * time / 2;

	process_noise(time);
	return pred.to_vec();
}

void EkfModel::update_rot_mats(float theta_x, float theta_y, float theta_z) {
	auto cosx = std::cos(theta_x);
	auto sinx = std::sin(theta_x);
	Rx(1,1) = cosx;
	Rx(1,2) = -sinx;
	Rx(2,1) = sinx;
	Rx(2,2) = cosx;
	auto cosy = std::cos(theta_y);
	auto siny = std::sin(theta_y);
	Ry(0,0) = cosy;
	Ry(0,2) = siny;
	Ry(2,0) = -siny;
	Ry(2,2) = cosy;
	auto cosz = std::cos(theta_z);
	auto sinz = std::sin(theta_z);
	Rz(0,0) = cosz;
	Rz(0,1) = -sinz;
	Rz(1,0) = sinz;
	Rz(1,1) = cosz;
}

EkfModel::Vec3 EkfModel::apply_rotation(const Vec3 &x) {
	return Rz * Ry * Rx * x;
}

void EkfModel::process_noise(float time) {
	R(0, 0) = time * 0.001f;
	R(1, 1) = time * 0.001f;
	R(2, 2) = time * 0.00001f;
	R(3, 3) = time * 0.0001f;
	R(4, 4) = time * 0.0001f;
	R(5, 5) = time * 0.00001f;
	R(6, 6) = time * 0.0001f;
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
	z(4, 0) = 0;
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

	Rx.setZero();
	Rx(0,0) = 1;
	Ry.setZero();
	Ry(1,1) = 1;
	Rz.setZero();
	Rz(2,2) = 1;
}
