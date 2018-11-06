#ifndef VSSS_CONTROL_H
#define VSSS_CONTROL_H

#include "SensorFusion.h"
#include "Controller.h"
#include "VFO.h"
#include "ApdsSensor.h"

struct PolarPose {
	float error;
	float theta;
	float alpha;
};

enum class ControlState {
	Pose, Position, Vector, Orientation, AngularVel, SeekBall, None
};

class Control {
	public:
	Controller controller;
	SensorFusion sensors;
	VFO vfo;

	ApdsSensor *back_apds;
	ApdsSensor *front_apds;

	Thread control_thread;

	volatile ControlState state = ControlState::None;
	bool stop_afterwards = true;

	Target target{{0, 0}, 0, 0};

	Timer backwards_timer;
	bool backwards = false;

	bool sleep_enabled = true;

	float k1 = 1;
	float k2 = 6;
	float uvf_n = 0.4;
//	float B = 0.006;

	explicit Control();
	void start_threads();
	void resume_threads();
	void reset_timeout();

	void stop_and_sleep();
	void set_ekf_vision_data(float x, float y, float theta);
	void set_target(ControlState control_type, Target target, bool stop_afterwards);
	bool backwards_select(float theta_error);

	void pose_control_thread();

	TargetVelocity pose_control(Pose pose, Target target);
	TargetVelocity position_control(Pose pose, Target target);
	TargetVelocity seek_ball(Pose pose, Target target);
	TargetVelocity run_to_ball(Pose pose, Target target);

	TargetVelocity uvf(Pose pose, Target target);
	TargetVelocity control_law(PolarPose pose, float vmax);
	TargetVelocity vector_control(float theta, float target_theta,
								  float velocity, bool enable_backwards = true);
	WheelVelocity get_target_wheel_velocity(TargetVelocity target);
	PolarPose get_polar_pose(Pose pose, Target target);
	TargetVelocity orientation_control(Pose pose, float theta);
	TargetVelocity set_stop_and_sleep();
};

#endif //VSSS_CONTROL_H
