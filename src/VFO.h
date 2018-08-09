#ifndef VSSS_VFO_H
#define VSSS_VFO_H

#include "SensorFusion.h"
#include "Controller.h"
#include "helper_functions.h"

struct H {
	float h1;
	Vector hs;
};

struct TargetPose {
	Point position;
	float theta;
};

struct TargetVelocity {
	float v;
	float w;
};

class VFO {
	public:
		Controller controller;
		SensorFusion sensors;

		Thread control_thread;

		TargetPose target{{0, 0.1}, 0};

		explicit VFO();
		void start_threads();
		void set_target_pose(float x, float y, float theta);
		void pose_control_thread();
		WheelVelocity get_target_wheel_velocity(TargetVelocity target) const;

//		VFO parameters
		float n = 1;
		float k1 = 1.5;
		float kp = 2;
		int direction = 1;

//		VFO functions
		TargetVelocity control_law(TargetPose target);
		H convergence_field(TargetPose target);
		Vector position_convergence_field_dot(TargetPose target);
		Vector ref_velocity(TargetPose target);
		Vector ref_velocity_dot(TargetPose target);
		float ref_theta(Vector hs);
		float ref_theta_dot(Vector hs, TargetPose target);
		Vector position_error(Point target);
		float orientation_error(Vector hs);
		Vector position_error_dot();
};

#endif //VSSS_VFO_H
