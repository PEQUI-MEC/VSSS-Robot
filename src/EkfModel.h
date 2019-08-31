#ifndef VSSS_EKFMODEL_H
#define VSSS_EKFMODEL_H

#include <Types.h>

const float ROBOT_SIZE = 0.0675f;

class EkfModel {
	private:
	void process_noise(float time);

	public:
	using EKF = EKFTypes<Pose::SIZE, SensorData::SIZE, VisionData::SIZE, Controls::SIZE>;
	using RotMat = Eigen::Matrix3f;
	using Vec3 = Eigen::Vector3f;

	EKF::PoseMat F;
	EKF::PoseMat R;

	EKF::HSensorMat H;
	EKF::SensorMat Q;

	EKF::HVisionMat Hv;
	EKF::VisionMat Qv;

	float ang_acc_z;
	float ang_acc_y;

	RotMat Rx;
	RotMat Ry;
	RotMat Rz;

	EkfModel();
	EKF::PoseVec prediction(const EKF::PoseVec &prev_x,
							const Controls &controls, float time);
	EKF::SensorVec sensor_measurement_error(const EKF::PoseVec &x, const EKF::SensorVec &z);
	EKF::SensorVec sensor_measurement_model(const EKF::PoseVec &x);
	EKF::VisionVec vision_measurement_error(const EKF::PoseVec &x, const EKF::VisionVec &z);
	EKF::VisionVec vision_measurement_model(const EKF::PoseVec &x);
	Vec3 apply_rotation(const Vec3 &x);
	void update_rot_mats(float theta_x, float theta_y, float theta_z);
	void use_magnetometer(bool use);
	void use_encoders(bool use);
};

#endif //VSSS_EKFMODEL_H
