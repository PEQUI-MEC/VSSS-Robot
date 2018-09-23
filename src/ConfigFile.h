#ifndef VSSS_CONFIGSFILE_H
#define VSSS_CONFIGSFILE_H

#include <string>
#include <mbed.h>

class ConfigFile {
	private:
		LocalFileSystem local;
		const std::string filename;
		FILE *file;

	public:
		explicit ConfigFile(const std::string &file_path);
		~ConfigFile();
		std::string get_data(const std::string &name, bool optional = false);
		float get_float(const std::string &name);
		void set_data(const std::string &name, const std::string &data);
		uint16_t get_xbee_addr();
};

#endif //VSSS_CONFIGSFILE_H
