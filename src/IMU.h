#ifndef IMU_H
#define IMU_H

#include "mbed.h"
#include "IMU_regs.h"

class IMU {
		I2C *i2c;
		int addr_gyro_acc;
		int addr_comp;

	public:
		void init(PinName sda, PinName scl);
		void read_acc(int16_t *data);
		void read_gyro(int16_t *data);
		void read_comp(int16_t *data);
		void read_reg(int addr, uint8_t reg, char *data, int num_bytes);
		void write_reg(int addr, uint8_t reg, uint8_t data);
};

#endif

