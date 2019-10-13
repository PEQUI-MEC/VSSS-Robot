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

	float last_gyro_z = 0;
	float last_vel_x = 0;

	EkfModel();
	EKF::PoseVec prediction(const EKF::PoseVec &prev_x, const Controls &controls,
							const WheelVelocity &wheel_vel, float time);
	EKF::SensorVec sensor_measurement_error(const EKF::PoseVec &x, const EKF::SensorVec &z);
	EKF::SensorVec sensor_measurement_model(const EKF::PoseVec &x);
	EKF::VisionVec vision_measurement_error(const EKF::PoseVec &x, const EKF::VisionVec &z);
	EKF::VisionVec vision_measurement_model(const EKF::PoseVec &x);
	void use_magnetometer(bool use);
	void use_encoders(bool use);
};

#endif //VSSS_EKFMODEL_H
