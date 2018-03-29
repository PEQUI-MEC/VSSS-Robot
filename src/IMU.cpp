#include "IMU.h"

// Configura os sensores
void IMU::init(PinName sda, PinName scl) {
	addr_gyro_acc = 0b1101011 << 1;
	addr_comp = 0b0011110 << 1;

	i2c = new I2C(sda, scl);
	// Habilita acelerometro nos 3 eixos
	write_reg(addr_gyro_acc, CTRL9_XL, 0x38);

	// Acc no modo de alta performance
	write_reg(addr_gyro_acc, CTRL1_XL, 0x60);

	// Habilita giroscopio nos 3 eixos
	write_reg(addr_gyro_acc, CTRL10_C, 0x38);

	// Gyro no modo de alta performance
	write_reg(addr_gyro_acc, CTRL2_G, 0x6C);

	// Habilita bussola
	write_reg(addr_comp, LIS3MDL_CTRL_REG2, 0x40);
	write_reg(addr_comp, LIS3MDL_CTRL_REG1, 0xFC);
	write_reg(addr_comp, LIS3MDL_CTRL_REG4, 0x0C);
	write_reg(addr_comp, LIS3MDL_CTRL_REG3, 0x00);
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas do acelerometro
void IMU::read_acc(int16_t *data) {
	read_reg(addr_gyro_acc, OUTX_L_XL, (char *) data, 6);
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas do giroscopio
void IMU::read_gyro(int16_t *data) {
	read_reg(addr_gyro_acc, OUTX_L_G, (char *) data, 6);
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas da bussola
void IMU::read_comp(int16_t *data) {
	read_reg(addr_comp, LIS3MDL_OUT_X_L, (char *) data, 6);
}

// Preenche o vetor data com os valores lidos do registrador, num_bytes indica quantos bytes devem ser lidos
void IMU::read_reg(int addr, uint8_t reg, char *data, int num_bytes) {
	i2c->write(addr, (char *) &reg, 1, true);
	i2c->read(addr, data, num_bytes);
}

// Escreve em um registrador
void IMU::write_reg(int addr, uint8_t reg, uint8_t data) {
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = data;
	i2c->write(addr, (char *) cmd, 2);
}

