#ifndef VSSS_UKF_H
#define VSSS_UKF_H

#include "EkfModel.h"
#include "Types.h"

class UKF {
	public:
	EkfModel model;
	using T = EkfModel::EKF;

	T::PoseVec x{};
	T::PoseMat COV{};
	T::UKFSigmaMat X{};

	bool new_log = false;
	float x_error = 0;
	float y_error = 0;
	float theta_error = 0;

	static constexpr float alpha = 1.5;
	static constexpr float k = 0;
	static constexpr int L = 6;

	static constexpr float lambda = std::pow(alpha, 2.0f) * (L + k) - L;
	static constexpr float weight0_m = lambda / (L + lambda);
	static constexpr float weight0_c = lambda / (L + lambda) + (3 - std::pow(alpha, 2.0f));
	static constexpr float weight = 1 / (2 * (L + lambda));

	UKF();

	float get_weight(int i, bool m);

	void set_sigma_points(const T::PoseVec &x);

	void predict(const T::ControlVec &controls, float time);

	void update_on_sensor_data(const T::SensorVec &data);

	void update_on_vision_data(const T::VisionVec &data);
};

#endif //VSSS_UKF_H
