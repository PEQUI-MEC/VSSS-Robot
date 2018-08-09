#ifndef VSSS_ADPSSENSOR_H
#define VSSS_ADPSSENSOR_H

#include <mbed.h>
#include <AdpsRegs.h>
#include <string>

struct ColorRGBC {
	uint16_t r;
	uint16_t g;
	uint16_t b;
	uint16_t c;

	std::string to_string() {
		return std::string("r: ") + std::to_string(r) +
				", g: " + std::to_string(g) +
				", b: " + std::to_string(b) +
				", c: " + std::to_string(c) + '\n';
	}
};

struct Location {
	uint8_t up;
	uint8_t down;
	uint8_t left;
	uint8_t right;

	std::string to_string() {
		return std::string("left: ") + std::to_string(left) +
				", right: " + std::to_string(right) + '\n';
	}
};

class AdpsSensor {
	private:
		I2C i2c;
		static constexpr uint16_t addr = 0x39 << 1;

	public:
		AdpsSensor(PinName sda, PinName scl);
		uint8_t read_proximity();
		ColorRGBC read_color();
		Location read_location();
		void read_reg(AdpsReg reg, uint8_t* data, int num_bytes);
		void write_reg(AdpsReg reg, uint8_t data);
};

#endif //VSSS_ADPSSENSOR_H
