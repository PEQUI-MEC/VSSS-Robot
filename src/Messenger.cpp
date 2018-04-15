#include <sstream>
#include "Messenger.h"
#include "PIN_MAP.h"

#define PI 3.141592653589793238

using std::string;

void Messenger::send_msg(string msg, uint16_t addr) {
	XBeeLib::RemoteXBee802 remoteDevice = XBeeLib::RemoteXBee802(addr);
	xbee->send_data(remoteDevice, (const uint8_t *) msg.c_str(), (uint16_t ) msg.size(), false);
}

bool Messenger::get_tokens(string &msg, int size, unsigned int first_char_position) {
	unsigned int pos_atual = first_char_position;
	for (int i = 0; i < size; ++i) {
		size_t delim_pos = msg.find(';', pos_atual);

		if (delim_pos == string::npos && i != size - 1)
			return false;

		tokens[i] = msg.substr(pos_atual, delim_pos - pos_atual);
		pos_atual = delim_pos + 1;
	}
	return true;
}

void Messenger::Update_PID_Pos(string msg) {
	if (!get_tokens(msg, 2, 1)) return;
	float kgz = float(atof(tokens[0].c_str()));
	float max_theta_error = float(atof(tokens[1].c_str()));

	robot->kgz = kgz;
	robot->set_max_theta_error(max_theta_error);
}

void Messenger::Update_PID_K(string msg) {
	if (msg[1] == 'P') {
		Update_PID_Pos(msg.substr(1));
		return;
	}

	if (!get_tokens(msg, 3, 1)) return;
	float kp = float(atof(tokens[0].c_str()));
	float ki = float(atof(tokens[1].c_str()));
	float kd = float(atof(tokens[2].c_str()));

	robot->controller.set_pid_constants(kp,ki,kd);
}

void Messenger::GoToPoint(string msg) {
	if (!get_tokens(msg, 3, 1)) return;
	float px = float(atof(tokens[0].c_str()));
	float py = float(atof(tokens[1].c_str()));
	float v = float(atof(tokens[2].c_str()));

	robot->start_position_control(px, py, v, true);
}

void Messenger::GoToVector(string msg) {
	if (!get_tokens(msg, 2, 1)) return;
	float theta = float(atof(tokens[0].c_str()));
	float v = float(atof(tokens[1].c_str()));

	robot->start_vector_control(theta, v, true);
}

void Messenger::Update_ACC(string msg) {
	if (!get_tokens(msg, 1, 1)) return;
	robot->acc_rate = float(atof(tokens[0].c_str()));
}

void Messenger::goToOrientation(string msg) {
	if (!get_tokens(msg, 2, 1)) return;

	float desiredAng = float(atof(tokens[0].c_str()));
	float vel = float(atof(tokens[1].c_str()));

	robot->start_orientation_control(desiredAng, vel, true);
}

string Messenger::decode_strings(string msg) {
	size_t id_pos, end_pos;
	if ((id_pos = msg.find(ID)) == string::npos ||
		(end_pos = msg.find('#', id_pos)) == string::npos)
		return "";
	else return msg.substr(id_pos + 2, end_pos - id_pos - 2);
}

void Messenger::send_battery() {
	AnalogIn vin_all_cells(ALL_CELLS);
	double vbat = vin_all_cells.read() * (3.3 * 1470 / 470);
	char buffer[8];
	gcvt(vbat,4,buffer);
	string msg_bat = "B"+string(buffer);
	send_msg(msg_bat);
}

void Messenger::decode_msg(string msg) {
	if (msg[1] == '@') {
		string s = decode_strings(msg);
		if (!s.empty()) msg = s;
		else return;
	}

	switch (msg[0]) {
		case 'K':
			//MENSAGEM DE CALIBRACAO DO PID
			Update_PID_K(msg);
			return;
		case 'A':
			//MENSAGEM DE CALIBRACAO DA ACELERACAO
			Update_ACC(msg);
			return;
		case 'O':
			//MENSAGEM DE COMANDO PARA CONTROLE DE ORIENTACAO
			goToOrientation(msg);
			return;
		case 'P':
			//MODO DE NAVEGACAO POR PONTOS
			GoToPoint(msg);
			return;
		case 'V':
			//MODO DE NAVEGACAO POR VETORES
			GoToVector(msg);
			return;
		case 'D':
			debug_mode = !debug_mode;
//			(*r).controllerA->debug_mode = debug_mode;
//			(*r).controllerB->debug_mode = debug_mode;
//          s->printf("Debug mode %d \n",debug_mode);
			return;
		case 'B':
			send_battery();
			return;
		default:
			break;
	}

	if (!get_tokens(msg, 2, 0)) return;
	float vel_right = float(atof(tokens[0].c_str()));
	float vel_left = float(atof(tokens[1].c_str()));
	robot->start_velocity_control(vel_left, vel_right);
}

Messenger::Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee) {
	xbee = this_xbee;
	debug_mode = false;
	this->robot = robot;
	ID = id;
}
