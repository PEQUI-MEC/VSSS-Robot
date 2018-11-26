#ifndef VSSS_UKF_H
#define VSSS_UKF_H

#include <Sparse>
#include <Eigen>
#include <Core>
#include "EkfModel.h"
#include "Types.h"
#include <MatrixFunctions>

class UKF {
	public:
	EkfModel model;
	using T = EkfModel::EKF;

	T::PoseVec x{};
	T::PoseMat COV{};
	T::UKFSigmaMat X{};

	static constexpr float alpha = 1;
	static constexpr float k = 0.1;
	static constexpr int L = 5;

	const float lambda = std::pow(alpha, 2.0f) * (L + k) - L;
	const float weight0_m = lambda / (L + lambda);
	const float weight0_c = lambda / (L + lambda) + (3 - std::pow(alpha, 2.0f));
	const float weight = 1 / (2 * (L + lambda));

	UKF() {
		x.setZero();
		COV.setIdentity();
		X.setZero();
	}

	float get_weight(int i, bool m) {
		if (i == 0 && m) return weight0_m;
		else if (i == 0) return weight0_c;
		else return weight;
	}

	void set_sigma_points(const T::PoseVec &x) {
		const T::PoseMat temp = (L + lambda) * COV;
		const T::PoseMat sqrt_mat = temp.sqrt();
//		const T::PoseMat &sqrt_mat = temp;
		X.col(0) = x;
		for (int i = 1; i <= L; i++)
			X.col(i) = x + sqrt_mat.row(i - 1).transpose();
		for (int i = L + 1; i <= 2 * L; i++)
			X.col(i) = x - sqrt_mat.row(i - L - 1).transpose();
	}

	void predict(const T::ControlVec &controls, float time) {
//		Predicted sigma points
		set_sigma_points(x);
		for (int i = 0; i <= 2 * L; i++)
			X.col(i) = model.prediction(X.col(i), controls, time);
//		Predicted pose
		T::PoseVec x_predicted;
		x_predicted.setZero();
		for (int i = 0; i <= 2 * L; i++)
			x_predicted += get_weight(i, true) * X.col(i);
//		Predicted covariance
		T::PoseMat COV_predicted;
		COV_predicted.setZero();
		for (int i = 0; i <= 2 * L; i++) {
		    T::PoseVec error = X.col(i) - x_predicted;
			COV_predicted += get_weight(i, false) * error * error.transpose();
		}
//		Update pose and covariance
        x_predicted(2, 0) = wrap(x_predicted(2, 0));
		x = x_predicted;
		COV = COV_predicted + model.R;
	}

	void update_on_sensor_data(const T::SensorVec &data) {
//		Predicted measurement sigma points
//	    set_sigma_points(x);
		T::UKFSensorSigmaMat Y;
		for (int i = 0; i <= 2 * L; i++)
			Y.col(i) = model.sensor_measurement_model(X.col(i));
//		Predicted measurement
		T::SensorVec y_predicted;
		y_predicted.setZero();
		for (int i = 0; i <= 2 * L; i++)
		    y_predicted += get_weight(i, true) * Y.col(i);
//		Predicted covariances
		T::SensorMat COV_YY;
		T::KSensorMat COV_XY;
		COV_YY.setZero();
		COV_XY.setZero();
		for (int i = 0; i <= 2 * L; i++) {
			T::SensorVec y_error = Y.col(i) - y_predicted;
			T::PoseVec x_error = X.col(i) - x;
			COV_YY += get_weight(i, false) * y_error * y_error.transpose();
			COV_XY += get_weight(i, false) * x_error * y_error.transpose();
		}
		COV_YY += model.Q;
//		Update pose and covariance
		T::KSensorMat K_GAIN = COV_XY * COV_YY.inverse();
		T::SensorVec error = data - y_predicted;
		error(0, 0) = 0;
//		error(2, 0) = 0;
//		error(3, 0) = 0;
		x = x + K_GAIN * error;
		COV = COV - K_GAIN * COV_YY * K_GAIN.transpose();
	}
};

#endif //VSSS_UKF_H
