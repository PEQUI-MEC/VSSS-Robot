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

float ConfigFile::get_float(const string &name) {
	return std::stof(get_data(name, false));
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
			string new_line = name;
			new_line += ':';
			new_line += data;
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
