#include "Robot.h"
#include <cmath>

using std::abs;

#define PI 3.1415265358

// **********************CONSTRUTOR ********************


Robot::Robot() :
		M_A_1(MOTOR_A_PIN_1),
		M_A_2(MOTOR_A_PIN_2),
		M_B_1(MOTOR_B_PIN_1),
		M_B_2(MOTOR_B_PIN_2) {
}

// ********************************* ********************
void Robot::init() {
	Vector_Control = false;
	Vector_msg_timeout.start();
	LOS_ant = 0;
	LOS_acc = 0;
	old_vel = 0;
	MAX_Theta_Error = 20;
	// Configuracao do periodo do pwm em 2ms (frequencia de 500Hz)
	acc_posA = 0;
	acc_posB = 0;
	desiredVr = 0;
	desiredVl = 0;
	ACC_RATE = 1;
	M_A_1.period_ms(2);
	M_A_2.period_ms(2);
	M_B_1.period_ms(2);
	M_B_2.period_ms(2);
	controllerA = new Controller();
	controllerB = new Controller();
	controllerA->init(&acc_posA, &desiredVr, TACHPIN_A_1, TACHPIN_A_2, PULSES_REVOLUTION, MSG_TIMEOUT, CONTROL_LOOP_MS);
	controllerB->init(&acc_posB, &desiredVl, TACHPIN_B_1, TACHPIN_B_2, PULSES_REVOLUTION, MSG_TIMEOUT, CONTROL_LOOP_MS);
	goToActive = false;
	Orientation_Control = false;
	xr = 0;
	yr = 0;
	erro = 0;
	erro_ant = 0;
	theta = 0;
	targetTheta = 0;
	acc_posA = 0;
	acc_posB = 0;
	vel_acelerada = 0;
	PID_POS[0] = 2.5;
	PID_POS[1] = 0;
	PID_POS[2] = 0.5;
	desiredVlin = 0;
	desiredVang = 0;
	desiredPos[0] = 0;
	desiredPos[1] = 0;
	t_ControlPos = new Thread();
	t_ControlPos->start(callback(Control_Pos, this));
	t_ControlVel = new Thread();
	t_ControlVel->start(callback(Control_Vel, this));
	desiredOrientation = 0;
	kgz = 1;
}

void Robot::goToOrientation(float targetTheta, float vel) {

	float m, LOS;
	theta = theta + (acc_posA - acc_posB) / Largura_Robo;
	targetTheta = targetTheta * PI / 180;
	targetTheta = atan2(sin(targetTheta), cos(targetTheta));
	if (((atan2(sin(targetTheta - theta + PI / 2), cos(targetTheta - theta + PI / 2)))) < 0) {
		backward = true;
		m = -1;
	} else {
		backward = false;
		m = 1;
	}

	if (backward) {
		double atheta = theta + PI;
		atheta = atan2(sin(atheta), cos(atheta));
		LOS = atan2(sin(targetTheta - atheta), cos(targetTheta - atheta));
	} else {
		LOS = atan2(sin(targetTheta - theta), cos(targetTheta - theta));
	}
	float Kp = 1;
	Vr = Kp * LOS;
	Vl = -Kp * LOS;

	if (abs(Vl) > 1) {
		Vl = 1 * Vl / abs(Vl);
	}
	if (abs(Vr) > 1) {
		Vr = 1 * Vr / abs(Vr);
	}

	Vl *= vel;
	Vr *= vel;

	//d=100;
	if (abs(LOS) < 2 * PI / 180) {
		desiredVr = 0;
		desiredVl = 0;
		Orientation_Control = false;
		controllerB->start_timer_new_msg();
		controllerA->start_timer_new_msg();
		//serial->printf("%d | %.2f ;%.2f; %.2f %.2f\n",backward,xr,yr,theta,targetTheta);//,Vr,Vl,targetTheta);
		acc_posA = 0;
		acc_posB = 0;
		return;
	}

	desiredVl = Vl;
	desiredVr = Vr;
	controllerB->start_timer_new_msg();
	controllerA->start_timer_new_msg();
	//serial->printf("%.2f \n",theta*180/PI);
	acc_posA = 0;
	acc_posB = 0;
}

void Robot::Control_Pos(void const *arg) {
	Robot *self = (Robot *) arg;
	//self->serial->printf("Tread ON\n");
	while (1) {
		if (self->goToActive) {
			//self->serial->printf("Going to target %.2f %.2f %.2f\n",self->desiredPos[0],self->desiredPos[1],self->desiredVlin);
			//goToTarget_PID(messenger.desiredPos[0], messenger.desiredPos[1], messenger.desiredVlin,messenger.ACC_RATE);
			self->goToTarget_SIN(self->desiredPos[0], self->desiredPos[1], self->desiredVlin);
		} else if (self->Orientation_Control) {

			self->goToOrientation(self->desiredOrientation, self->desiredVang);
		} else if (self->Vector_Control) {

			self->goToVector(self->desiredOrientation, self->desiredVlin);
		} else {
			// self->serial->printf("Stopping\n");
			self->xr = 0;
			self->yr = 0;
			self->erro = 0;
			self->erro_ant = 0;
			self->theta = 0;
			self->targetTheta = 0;
			self->acc_posA = 0;
			self->acc_posB = 0;
			self->vel_acelerada = 0;
		}
		//wait(float(POS_LOOP)/1000.0);
		Thread::wait(POS_LOOP);
	}
}

// ********************************* CONTROLADOR DE POSICAO **************************************
void Robot::goToVector(float theta_arg, float vel) {

	if (vel_acelerada < 0.3)
		vel_acelerada = 0.3;

	if (vel == 0) {
		vel_acelerada = 0;
		desiredVr = 0;
		desiredVl = 0;
		goToActive = false;
		controllerB->start_timer_new_msg();
		controllerA->start_timer_new_msg();
		//serial->printf("%d | %.2f ;%.2f; %.2f %.2f\n",backward,xr,yr,theta,targetTheta);//,Vr,Vl,targetTheta);
		acc_posA = 0;
		acc_posB = 0;
		return;
		//d=100;
	}



	//if(Dv<-0.2)
	//  vel_acelerada = vel_acelerada - ACC_RATE*POS_LOOP/1000;
	float m, LOS;
	pos = (acc_posA + acc_posB) / 2;
	xr = xr + pos * cos(theta);
	yr = yr + pos * sin(theta);
	targetTheta = (atan2(desiredPos[1] - yr, desiredPos[0] - xr));

	theta = theta + (acc_posA - acc_posB) / Largura_Robo;

	if (((atan2(sin(targetTheta - theta + PI / 2), cos(targetTheta - theta + PI / 2)))) < 0) {
		if (!backward) {
			vel_acelerada = 0.3;
		}
		backward = true;
		m = -1;
	} else {
		if (backward) {
			vel_acelerada = 0.3;
		}

		backward = false;
		m = 1;
	}

	if (abs(90 - theta_arg) < 20) backward = false;

	if (backward) {
		double atheta = theta + PI;
		atheta = atan2(sin(atheta), cos(atheta));
		LOS = atan2(sin(targetTheta - atheta), cos(targetTheta - atheta));
	} else {
		LOS = atan2(sin(targetTheta - theta), cos(targetTheta - theta));
	}

	if (abs(LOS) > MAX_Theta_Error * PI / 180) {
		vel_acelerada = vel_acelerada - 2 * ACC_RATE * POS_LOOP / 1000;
	} else {
		//serial->printf("ENTREI\n");
		float Dv = vel - vel_acelerada;
		if (Dv > 0.2) {
			vel_acelerada = vel_acelerada + ACC_RATE * POS_LOOP / 1000;
		} else if (Dv < 0) {
			vel_acelerada = vel;
		}
	}
	Vr = (m + sin(LOS) + m * kgz * tan(m * LOS / 2));
	Vl = (m - sin(LOS) + m * kgz * tan(-m * LOS / 2));

	if (abs(Vl) > 1) {
		Vl = 1 * Vl / abs(Vl);
	}
	if (abs(Vr) > 1) {
		Vr = 1 * Vr / abs(Vr);
	}

	Vl *= vel_acelerada;
	Vr *= vel_acelerada;
	acc_posA = 0;
	acc_posB = 0;
	controllerB->start_timer_new_msg();
	controllerA->start_timer_new_msg();

	//d=100;
	if (Vector_msg_timeout.read_ms() > MSG_TIMEOUT) {
		desiredVr = 0;
		desiredVl = 0;
		Vector_Control = false;
	} else {
		desiredVl = Vl;
		desiredVr = Vr;
	}

	//serial->printf("%d | %.2f ;%.2f; %.2f %.2f\n",backward,xr,yr,theta,targetTheta);//,Vr,Vl,targetTheta);

}

void Robot::goToTarget_SIN(double x, double y, double vel) {
	//serial->printf("Going to target %.2f %.2f %.2f\n",x,y,vel);
	//serial->printf("goToTarget\n");
	//kgz = -(1 + sin(MAX_Theta_Error))/(tan(MAX_Theta_Error/2);
	float d = sqrt(pow(xr - x, 2) + pow(yr - y, 2));
	if (vel_acelerada < 0.3)
		vel_acelerada = 0.3;

	if (vel == 0) {
		vel_acelerada = 0;
		desiredVr = 0;
		desiredVl = 0;
		goToActive = false;
	}



	//if(Dv<-0.2)
	//  vel_acelerada = vel_acelerada - ACC_RATE*POS_LOOP/1000;
	float m, LOS;
	pos = (acc_posA + acc_posB) / 2;
	xr = xr + pos * cos(theta);
	yr = yr + pos * sin(theta);
	targetTheta = (atan2(y - yr, x - xr));

	theta = theta + (acc_posA - acc_posB) / Largura_Robo;

	if (((atan2(sin(targetTheta - theta + PI / 2), cos(targetTheta - theta + PI / 2)))) < 0) {
		if (!backward) {
			vel_acelerada = 0.3;
		}
		backward = true;
		m = -1;
	} else {
		if (backward) {
			vel_acelerada = 0.3;
		}
		backward = false;
		m = 1;
	}

	if (backward) {
		double atheta = theta + PI;
		atheta = atan2(sin(atheta), cos(atheta));
		LOS = atan2(sin(targetTheta - atheta), cos(targetTheta - atheta));
	} else {
		LOS = atan2(sin(targetTheta - theta), cos(targetTheta - theta));
	}

	if (abs(LOS) > MAX_Theta_Error * PI / 180) {
		if (vel_acelerada > 0.8)
			vel_acelerada = 0.8;
		else if (vel_acelerada > 0.3)
			vel_acelerada = vel_acelerada - 2 * ACC_RATE * POS_LOOP / 1000;
	} else {
		//serial->printf("ENTREI\n");
		float Dv = vel - vel_acelerada;
		if (Dv > 0.2)
			vel_acelerada = vel_acelerada + ACC_RATE * POS_LOOP / 1000;
	}
	vel = vel_acelerada;
	float limiar = atan2(1, d);
	limiar = limiar > 30 ? 30 : limiar;
	if (abs(LOS) < limiar)
		LOS = 0;
	Vr = (m + sin(LOS) + m * kgz * tan(m * LOS / 2));
	Vl = (m - sin(LOS) + m * kgz * tan(-m * LOS / 2));

	if (abs(Vl) > 1) {
		Vl = 1 * Vl / abs(Vl);
	}
	if (abs(Vr) > 1) {
		Vr = 1 * Vr / abs(Vr);
	}

	Vl *= vel;
	Vr *= vel;



	//d=100;
	if (d < 1) {
		desiredVr = 0;
		desiredVl = 0;
		goToActive = false;
		return;
	}
	//if(d<10) {
	//    Vr = Vr*(d/10);
	//    Vl = Vl*(d/10);
	//}
	desiredVl = Vl;
	desiredVr = Vr;
	controllerB->start_timer_new_msg();
	controllerA->start_timer_new_msg();
	//serial->printf("%d | %.2f ;%.2f; %.2f %.2f\n",backward,xr,yr,theta,targetTheta);//,Vr,Vl,targetTheta);
	acc_posA = 0;
	acc_posB = 0;
}
// ********************************* CONTROLADOR DE VELOCIDADE **************************************

void Robot::Control_Vel(void const *arg) {
	Robot *self = (Robot *) arg;
	while (1) {
		self->controllerA->control_loop();
		self->controllerB->control_loop();

		float pwm = float(self->controllerA->PWM);
		if (self->desiredVr == 0) {
			//serial->printf("resetA");
			pwm = 0;
			self->controllerA->reset();
		}
		//pwm = (pwm/abs(pwm))*(1-abs(pwm)); // Correcao para modo slow-decay, calculo do pwm'

		// if(messenger.reinforcement_learning_mode)
		//   pwm = controllerA -> target_vel;

		if (pwm < 0) {
			pwm = (pwm / abs(pwm)) * (1 - abs(pwm));
			self->M_A_1.write(
					1); //   | write(pwm) + write(0) = fast decay -> mudanca brusca de corrente no pwm = frenagem mais potente
			self->M_A_2.write(
					-pwm);// | write(pwm) + wirte(1) = slow decay -> mudanca de corrente mais suave = relacao linear entre velocidade e pwm
		} else if (pwm > 0) {
			pwm = (pwm / abs(pwm)) * (1 - abs(pwm));
			self->M_A_1.write(pwm);
			self->M_A_2.write(1);
		} else {
			// frenagem em fast decay
			self->M_A_1.write(1);
			self->M_A_2.write(1);
		}

		pwm = float(self->controllerB->PWM);

		if (self->desiredVl == 0) {

			// serial->printf("resetB");
			pwm = 0;
			self->controllerB->reset();
		}
		// Correcao para modo slow-decay, calculo do pwm'

		//if(messenger.reinforcement_learning_mode)
		//   pwm = - controllerB -> target_vel;

		if (pwm < 0) {
			pwm = (pwm / abs(pwm)) * (1 - abs(pwm));
			self->M_B_1.write(
					1); //   | write(pwm) + write(0) = fast decay -> mudanca brusca de corrente no pwm = frenagem mais potente
			self->M_B_2.write(
					-pwm);// | write(pwm) + wirte(1) = slow decay -> mudanca de corrente mais suave = relacao linear entre velocidade e pwm

		} else if (pwm > 0) {
			pwm = (pwm / abs(pwm)) * (1 - abs(pwm));
			self->M_B_1.write(pwm);
			self->M_B_2.write(1);
		} else {
			// frenagem em fast decay
			self->M_B_1.write(1);
			self->M_B_2.write(1);
		}
	}
}
