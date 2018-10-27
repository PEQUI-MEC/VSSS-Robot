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
	T::PoseMat I{};

	static constexpr float alpha = 1;
	static constexpr float k = 0;
	static constexpr int L = 5;

	const float lambda = (std::pow(alpha, 2.0f) * L + k) - L;
	const float weight0 = lambda / L + lambda;
	const float weight = 1 / (2 * (L + lambda));

	T::PoseVec sigma(int i, const T::PoseVec &x, const T::PoseMat &sqrt_mat) {
		if (i <= L) return x + sqrt_mat.col(i);
		else return x - sqrt_mat.col(i - L);
	}

	T::SensorVec sigma_y(int i, const T::PoseMat &sqrt_mat) {
		return model.sensor_measurement_model(sigma(i, x, sqrt_mat));
	}

	void predict(const T::ControlVec &controls, float time) {
		const T::PoseMat temp = (L + lambda) * COV;
		const T::PoseMat sqrt_mat = temp.sqrt();
		T::PoseVec x_pred = weight0 * model.prediction(sigma(0, x, sqrt_mat), controls, time);
		for (int i = 1; i <= 2 * L; i++) {
			x_pred += weight * model.prediction(sigma(i, x, sqrt_mat), controls, time);
		}
		x = x_pred;

		const T::PoseVec error0 = sigma(0, x, sqrt_mat) - x;
		T::PoseMat COV_P = weight0 * error0 * error0.transpose();
		for (int i = 1; i <= 2 * L; i++) {
			const T::PoseVec error = sigma(i, x, sqrt_mat) - x;
			COV_P += weight * error * error.transpose();
		}
		COV = COV_P;
	}


	void update_on_sensor_data(const T::SensorVec &data) {
		const T::PoseMat temp = (L + lambda) * COV;
		const T::PoseMat sqrt_mat = temp.sqrt();
		T::SensorVec y = weight0 * sigma_y(0, sqrt_mat);
		for (int i = 1; i <= 2 * L; i++) {
			y += weight * sigma_y(1, sqrt_mat);
		}

		const T::SensorVec error0_y = sigma_y(0, sqrt_mat) - y;
		const T::PoseVec error0_x = sigma(0, x, sqrt_mat) - x;
		T::SensorMat COV_YY = weight0 * error0_y * error0_y.transpose();
		T::KSensorMat COV_XY = weight0 * error0_x * error0_y.transpose();

		for (int i = 1; i <= 2 * L; i++) {
			const T::SensorVec error_y = sigma_y(i, sqrt_mat) - y;
			const T::PoseVec error_x = sigma(i, x, sqrt_mat) - x;
			COV_YY += weight * error_y * error_y.transpose();
			COV_XY += weight * error_x * error_y.transpose();
		}
		T::KSensorMat K_GAIN = COV_XY * COV_YY;
		x = x + K_GAIN * (data - y);
		COV = COV - K_GAIN * COV_YY * K_GAIN.transpose();
	}
};

#endif //VSSS_UKF_H
