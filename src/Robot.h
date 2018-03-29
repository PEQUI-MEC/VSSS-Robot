#ifndef ROBOT_H_
#define ROBOT_H_

#include "mbed.h"
#include "Controller.h"
#include "PIN_MAP.h"
#include "Messenger.h"
class Messenger;

class Robot {
	public:
		char MY_ID;
		int MSG_TIMEOUT;
		float ACC_RATE;
		bool goToActive;
		bool Orientation_Control;
		float xr;
		float yr;
		float MAX_Theta_Error;
		double kgz;
		double pos;
		float acc_posA;
		float acc_posB;
		float desiredVr;
		float desiredVl;
		float desiredPos[2];
		float desiredVlin;
		float desiredOrientation;
		float desiredVang;
		float vel_acelerada;
		float theta;
		float PID_POS[3];
		float LOS_ant;
		float LOS_acc;
		float erro;
		float erro_ant;
		float targetTheta;
		float w, vlin, Vr, Vl;
		Messenger* messenger;
		bool backward;
		Controller *controllerA;
		Controller *controllerB;
		Thread *t_ControlVel;
		Thread *t_ControlPos;
		void init(Messenger* msgr);
		Timer Vector_msg_timeout;
		Robot();
		bool Vector_Control;
		//Robot() : M_A_1(MOTOR_A_PIN_1), M_A_2(MOTOR_A_PIN_2), M_B_1(MOTOR_B_PIN_1),M_B_2(MOTOR_B_PIN_2){};

		void goToVector(float, float);
		static void Control_Pos(void const *arg);
		void goToTarget_SIN(double x, double y, double vel);
		void goToTarget_PID(double x, double y, double vel);
		static void Control_Vel(void const *arg);
		void goToOrientation(float, float);

	private:
		float old_vel;
		PwmOut M_A_1;
		PwmOut M_A_2;
		PwmOut M_B_1;
		PwmOut M_B_2;
};

#endif 
