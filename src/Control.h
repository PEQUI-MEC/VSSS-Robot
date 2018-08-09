#ifndef VSSS_CONTROL_H
#define VSSS_CONTROL_H

#include "SensorFusion.h"
#include "Controller.h"

struct PolarPose {
	float error;
	float theta;
	float alpha;
};

struct TargetPose {
	float x;
	float y;
	float theta;
};

struct TargetVelocity {
	float v;
	float w;
};

class Control {
	public:
		Controller controller;
		SensorFusion sensors;

		Thread control_thread;

		TargetPose target{1, 1, 0};
		float gama = 1;
		float h = 1.5;
		float k = 1;

		float velocity = 0;
		float k1 = 1;
		float k2 = 8;
		float B = 0.006;
		bool swap = false;
//		float B = 0;

		explicit Control();
		void start_threads();
		void set_target_pose(float x, float y, float theta);
		void pose_control_thread();

		TargetVelocity control(TargetPose target);
		TargetVelocity control_law(PolarPose pose, float vmax) const;
		TargetVelocity vector_control(float target_theta, float velocity) const;
		WheelVelocity get_target_wheel_velocity(TargetVelocity target) const;
		PolarPose get_polar_pose(TargetPose target) const;
};

#endif //VSSS_CONTROL_H
