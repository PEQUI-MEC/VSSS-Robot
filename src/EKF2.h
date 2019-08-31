#ifndef VSSS_EKF2_H
#define VSSS_EKF2_H

#include <Sparse>
#include <Eigen>
#include <Core>

#include "EkfModel.h"
#include "Types.h"

class EKF2 {
	public:
	EkfModel model;
	using T = EkfModel::EKF;

	T::PoseVec x{};
	T::PoseMat COV{};
	T::PoseMat I{};

	EKF2() {
		I.setIdentity();
		COV.setIdentity();
		x.setZero();
	}

	void predict(const Controls &controls, float time) {
		x = model.prediction(x, controls, time);
		COV = model.F * COV * model.F.transpose() + model.R;
	}

	void update_on_sensor_data(const T::SensorVec &data) {
		auto error = model.sensor_measurement_error(x, data);
		T::SensorMat S = model.H * COV * model.H.transpose() + model.Q;
		T::KSensorMat K_GAIN = COV * model.H.transpose() * S.inverse();
		x = x + K_GAIN * error;
		COV = (I - K_GAIN * model.H) * COV;
	}
	
	void update_on_vision_data(const T::VisionVec &data) {
		auto error = model.vision_measurement_error(x, data);
		T::VisionMat S = model.Hv * COV * model.Hv.transpose() + model.Qv;
		T::KVisionMat K_GAIN = COV * model.Hv.transpose() * S.inverse();
		x = x + K_GAIN * error;
		COV = (I - K_GAIN * model.Hv) * COV;
	}
};

#endif //VSSS_EKF2_H
