#include "Controller.h"

#define PI 3.14159265358

void Controller::start_timer_new_msg() {
	msg_timeout.reset();
}

void Controller::set_target_velocity(float desired_vel) {
	*target_vel = desired_vel;
}

void Controller::resetPos() {
	*acc_pos = 0;
}

void Controller::reset() {
	currVel = 0;
	lastVel = 0;
	currErr = 0;
	*target_vel = 0;
	accErr = 0;
	PWM = 0;
}

void Controller::set_PID_constants(float KP, float KI, float KD) {
	kp = KP;
	ki = KI;
	kd = KD;
}

void Controller::init(float *acc_position, float *desiredVel, PinName INTERRUPT_PIN_1, PinName INTERRUPT_PIN_2, int PR,
					  int TO, int CL) {
	debug_mode = false;
	acc_pos = acc_position;
	target_vel = desiredVel;
	currVel = 0;
	lastVel = 0;
	currErr = 0;
	*target_vel = 0;
	*acc_pos = 0;
	accErr = 0;
	PWM = 0;
	kp = 1.26;
	ki = 0.0481;
	kd = 0;

	pulses_per_revolution = PR;
	Control_DT = CL;
	//QUADRATURE ENCODER INTERFACE PRONTA DO ARM
	wheel = new QEI(INTERRUPT_PIN_1, INTERRUPT_PIN_2, NC, PR, QEI::X4_ENCODING);
	wheel->reset();
	timer.start();
	msg_timeout.start();
	timeout = TO;
	sampling_timer.start();
}

void Controller::control_loop() {
	if (msg_timeout.read_ms() > timeout && !debug_mode) {
		// Mensagem expirada
		this->reset();
	}
	if (sampling_timer.read_ms() < Control_DT) {
		// Tempo de amostragem ainda não atingido
		return;
	} else {

		int curr_pulses = wheel->getPulses();
		DT = timer.read_ms();

		//12        ->  NUMERO DE PULSOS POR VOLTA
		//75.8126/1 ->  1 VOLTA DA RODA POR 75.8126 DO MOTOR ((NOMINAL 75/1))
		//1000      ->  TRANSFORMAEM MILISEGUNDOS PARA SEGUNDOS
		currVel = 1000 * float(curr_pulses) / (float(pulses_per_revolution) * 75.8126 * DT);
		currVel *= 0.06 * PI; // conversao pra m/s
		currErr = (*target_vel - currVel);
		accErr += currErr;

		//CALCULA P + I + D
		PID = currErr * kp + accErr * ki + (lastVel - currVel) * kd;
		lastVel = currVel;
		PWM = PID;

		//VALORES ACIMA DOS LIMITES SÃO ENQUADRADOS DENTRO DO LIMITE
		if (PWM > 1) PWM = 1;
		if (PWM < -1) PWM = -1;
		// Reset de variáveis para o proximo ciclo de controle
		timer.reset();
		sampling_timer.reset();
		wheel->reset();
		// Persistencia do acumulo de ticks do tacometro para odometria
		*acc_pos += 2 * PI * 3 * float(curr_pulses) / (float(pulses_per_revolution * 75.8126)); //em m
	}
}
