//
// Created by thiago on 15/07/18.
//

#ifndef VSSS_CONFIGSFILE_H
#define VSSS_CONFIGSFILE_H

#include "Robot.h"

class ConfigFile {
	private:
		LocalFileSystem local;
		const std::string filename;
		FILE *file;

	public:
		explicit ConfigFile(const std::string &file_path);
		~ConfigFile();
		void configure(Robot &robot, uint16_t &xbee_addr);
		std::string get_data(const std::string &name, bool optional = false);
		void set_data(const std::string &name, const std::string &data);
		void save_configs(const std::string &path, Robot &robot, uint16_t &xbee_addr);
};

#endif //VSSS_CONFIGSFILE_H
