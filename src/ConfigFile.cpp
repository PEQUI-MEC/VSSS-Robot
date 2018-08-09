#include "ConfigFile.h"

using std::string;


ConfigFile::ConfigFile(const std::string &file_path)
		: local("local"), filename(file_path) {
	file = fopen(file_path.c_str(), "r");
	MBED_ASSERT(file != nullptr);
}

ConfigFile::~ConfigFile() {
	fclose(file);
}

uint16_t ConfigFile::get_xbee_addr() {
	return (uint16_t) std::stoul(get_data("addr"), nullptr, 16);
}

void ConfigFile::configure(Robot &robot, uint16_t &xbee_addr) {
	xbee_addr = (uint16_t) std::stoul(get_data("addr"), nullptr, 16);
	robot.MY_ID = get_data("my_id")[0];
	robot.msg_timeout_limit = std::stoi(get_data("msg_timeout"));
	robot.acc_rate = std::stof(get_data("acc_rate"));
	robot.kgz = std::stof(get_data("kgz"));
	robot.max_theta_error = std::stof(get_data("kgz"));
}

string ConfigFile::get_data(const string &name, bool optional) {
	char line_buffer[32];
	while(fgets(line_buffer, sizeof(line_buffer), file)) {
		string line(line_buffer);
		size_t separator = line.find(':');
		if (line.substr(0, separator) == name) {
			rewind(file);
			return line.substr(separator + 1);
		}
	}
	MBED_ASSERT(optional);
	rewind(file);
	return string{};
}

void ConfigFile::set_data(const string &name, const string &data) {
	char line_buffer[32];
	bool found = false;
	const char * temp_file_name = "/local/temp.txt";
	FILE * temp_file = fopen(temp_file_name, "w");
	while(fgets(line_buffer, sizeof(line_buffer), file)) {
		string line(line_buffer);
		size_t separator = line.find(':');
		if (line.substr(0, separator) == name) {
			const string new_line = name + ':' + data;
			fputs(new_line.c_str(), temp_file);
			found = true;
		} else {
			fputs(line_buffer, temp_file);
		}
	}
	if(!found) {
		const string new_line = name + ':' + data;
		fputs(new_line.c_str(), temp_file);
	}
	remove(filename.c_str());
	rename(temp_file_name, filename.c_str());
	file = fopen(filename.c_str(), "r");
}

void ConfigFile::save_configs(const string &path, Robot &robot,
						   uint16_t &xbee_addr) {
	fprintf(file, "addr:%04x\n", xbee_addr);
	fprintf(file, "my_id:%c\n", robot.MY_ID);
	fprintf(file, "msg_timeout:%d\n", robot.msg_timeout_limit);
	fprintf(file, "acc_rate:%f\n", robot.acc_rate);
	fprintf(file, "kgz:%lf\n", robot.kgz);
	fprintf(file, "max_theta_error:%f\n", robot.max_theta_error);
}
