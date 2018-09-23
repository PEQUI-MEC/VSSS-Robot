#ifndef IMU_H
#define IMU_H

#include "mbed.h"
#include "IMU_regs.h"

struct imu_data {
	float gyro_z_rate;
	float mag_z_theta;
	float mag_x;
	float mag_y;
};

struct mag_components {
	float x;
	float y;
};

class IMU {
		I2C *i2c = nullptr;
		int addr_gyro_acc = 0b1101011 << 1;
		int addr_comp = 0b0011110 << 1;

		float gyro_scale = 1;

		float mag_max_x = 1;
		float mag_off_x = 1;
		float mag_max_y = 1;
		float mag_off_y = 1;

	public:
		IMU();
		void init(PinName sda, PinName scl);
		imu_data read_imu_data(bool use_mag);
		float read_mag();
		mag_components read_mag_components();
		float read_gyro();
		void read_acc_all(int16_t *data);
		void read_gyro_all(int16_t *data);
		void read_comp_all(int16_t *data);
		void read_reg(int addr, uint8_t reg, char *data, int num_bytes);
		void write_reg(int addr, uint8_t reg, uint8_t data);
};

#endif

