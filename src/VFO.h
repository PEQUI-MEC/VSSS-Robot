#ifndef VSSS_VFO_H
#define VSSS_VFO_H

#include "SensorFusion.h"
#include "Controller.h"
#include "helper_functions.h"
#include "Types.h"

struct H {
	float h1;
	Vector hs;
};

class VFO {
	public:
//	VFO parameters
	float n = 1.4;
	float k1 = 10;
	float kp = 2.2;
	int direction = 1;

//	VFO functions
	TargetVelocity control_law(Target target, const Pose &pose);
	H convergence_field(Target target, Pose pose);
	Vector position_convergence_field_dot(Target target, Pose pose);
	Vector ref_velocity(Target target, Point position);
	Vector ref_velocity_dot(Target target, Pose pose);
	float ref_theta(Vector hs);
	float ref_theta_dot(Vector hs, Target target, Pose pose);
	Vector position_error(Point position, Point target);
	float orientation_error(Vector hs, Pose pose);
	Vector position_error_dot(Pose pose);
};

#endif //VSSS_VFO_H
