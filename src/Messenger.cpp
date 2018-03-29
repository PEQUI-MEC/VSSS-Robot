#include <sstream>
#include "Messenger.h"

#define PI 3.141592653589793238

using std::string;

// ********************** SERIAL HANDLER AND MESSAGING FUNCTIONS *********************** //
void Messenger::send_msg(string msg, uint16_t addr) {
	XBeeLib::RemoteXBee802 remoteDevice = XBeeLib::RemoteXBee802(addr);
	xbee->send_data(remoteDevice, (const uint8_t *) msg.c_str(), (uint16_t ) msg.size(), false);
}

bool Messenger::get_tokens(string& msg, int size) {
	size_t pos_atual = 1;
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
	if (!get_tokens(msg, 2)) return;
	string kgz = tokens[0];
	string max_theta_error = tokens[1];

	(*r).kgz = atof(kgz.c_str());
	(*r).MAX_Theta_Error = atof(max_theta_error.c_str());
//  s->printf("ROBO %c KGZ | %f ; MAX_THETA| %f \n",MY_ID,(*r).kgz,(*r).MAX_Theta_Error);
}

void Messenger::Update_PID_K(string msg) {
	if (msg[1] == 'P') {
		Update_PID_Pos(msg.substr(1));
		return;
	}

	if (!get_tokens(msg, 3)) return;
	string kp = tokens[0];
	string ki = tokens[1];
	string kd = tokens[2];

//  CALIBRAR PID
	KPID[0] = atof(kp.c_str());
	KPID[1] = atof(ki.c_str());
	KPID[2] = atof(kd.c_str());

	(*r).controllerA->set_PID_constants(KPID[0], KPID[1], KPID[2]);
	(*r).controllerB->set_PID_constants(KPID[0], KPID[1], KPID[2]);
	//s->printf("KPID | %f | %f | %f\n",ctrlA->kp,ctrlA->ki,ctrlA->kd);
}

void Messenger::GoToPoint(string msg) {
	if (!get_tokens(msg, 3)) return;
	string px = tokens[0];
	string py = tokens[1];
	string v = tokens[2];

	(*r).xr = 0;
	(*r).yr = 0;
	(*r).theta = 0;
	(*r).acc_posA = 0;
	(*r).acc_posB = 0;
	(*r).desiredPos[0] = atof(px.c_str());
	(*r).desiredPos[1] = atof(py.c_str());
	(*r).desiredVlin = atof(v.c_str());
	(*r).goToActive = true;
//  s->printf("POS | %f | %f | %f\n",desiredPos[0],desiredPos[1],desiredVlin);
}

void Messenger::GoToVector(string msg) {
	if (!get_tokens(msg, 2)) return;
	string px = tokens[0];
	string v = tokens[1];

	(*r).xr = 0;
	(*r).yr = 0;
	(*r).theta = 0;
	(*r).acc_posA = 0;
	(*r).acc_posB = 0;
	(*r).desiredVlin = atof(v.c_str());
	(*r).desiredPos[1] = 50 * sin(atof(px.c_str()) * PI / 180);
	(*r).desiredPos[0] = 50 * cos(atof(px.c_str()) * PI / 180);
	(*r).Vector_msg_timeout.reset();
	(*r).Vector_Control = true;

//  s->printf("POS | %f | %f \n",(*r).desiredOrientation,(*r).desiredVlin);
}

void Messenger::Update_ACC(string msg) {
	if (!get_tokens(msg, 1)) return;
	string acc = tokens[0];

	(*r).ACC_RATE = atof(acc.c_str());
//  s->printf("ROBO %c ACC_RATE | %f \n",(*r).MY_ID,(*r).ACC_RATE);
}

void Messenger::goToOrientation(string msg) {
	if (!get_tokens(msg, 2)) return;
	string desiredAng = tokens[0];
	string vel = tokens[1];

	(*r).desiredOrientation = atof(desiredAng.c_str());
	(*r).desiredVang = atof(vel.c_str());
	(*r).Orientation_Control = true;
//  s->printf("ORIENTATION | %f | %f \n",msgToFloat[0],msgToFloat[1]);
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
			(*r).controllerA->debug_mode = debug_mode;
			(*r).controllerB->debug_mode = debug_mode;
//          s->printf("Debug mode %d \n",debug_mode);
			return;
		case 'B':
			send_battery();
			return;
		default:
			break;
	}

	if (!get_tokens(msg, 2)) return;
	string vel1 = tokens[0];
	string vel2 = tokens[1];

	desiredVel[0] = atof(vel1.c_str());
	desiredVel[1] = atof(vel2.c_str());
//  s->printf("desiredVel | %f | %f \n",desiredVel[0],desiredVel[1]);
	(*r).desiredVr = desiredVel[0];
	(*r).desiredVl = desiredVel[1];
	(*r).controllerA->start_timer_new_msg();
	(*r).controllerB->start_timer_new_msg();
}

Messenger::Messenger(char id, Robot *robot, XBeeLib::XBee802 *this_xbee) {
	xbee = this_xbee;
	bat_request = false;
	ACC_RATE = 0.7;
	reinforcement_learning_mode = false;
	debug_mode = false;
	goToActive = false;
	r = robot;
	ID = id;
	KPID[0] = 0;
	KPID[1] = 0;
	KPID[2] = 0;
	desiredVel[0] = 0;
	desiredVel[1] = 0;
}
