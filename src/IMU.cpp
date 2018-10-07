#include "IMU.h"

#define PI 3.1415926f

#define MAX_GYRO (2000 * (PI)/180) // 2000 dps para rad/s
#define MAX_MAG 4.0f
#define MAG_X_OFF 0.571852f
#define MAG_Y_OFF 0.132321f
#define MAG_X_MAX 0.174501f
#define MAG_Y_MAX 0.182615f

#define MAG_X_OFF_a 0.4261779f
#define MAG_Y_OFF_a 0.1183533f
#define MAG_X_MAX_a 0.1886219f
#define MAG_Y_MAX_a 0.1955043f

#define MAG_X_OFF_e 0.4261779f
#define MAG_Y_OFF_e 0.1183533f
#define MAG_X_MAX_e 0.1886219f
#define MAG_Y_MAX_e 0.1955043f

#define MAG_X_OFF_b 1.0294917f
#define MAG_Y_OFF_b (-0.24693936f)
#define MAG_X_MAX_b 0.1941387f
#define MAG_Y_MAX_b 0.2118146f

// Configura os sensores
void IMU::init(PinName sda, PinName scl) {
	addr_gyro_acc = 0b1101011 << 1;
	addr_comp = 0b0011110 << 1;

	i2c = new I2C(sda, scl);
	i2c->frequency(400*1000);
	// Habilita acelerometro nos 3 eixos
//	write_reg(addr_gyro_acc, CTRL9_XL, 0x38);

	// Acc no modo de alta performance
//	write_reg(addr_gyro_acc, CTRL1_XL, 0x60);

	// Habilita giroscopio nos 3 eixos
//	write_reg(addr_gyro_acc, CTRL10_C, 0x38);

	// Habilita giroscopio no eixo z
	write_reg(addr_gyro_acc, CTRL10_C, 0x20);

	// Gyro no modo de alta performance, 1.66KHz, 2000dps max
	write_reg(addr_gyro_acc, CTRL2_G, 0x7C);

//	Habilita BDU - Atualiza LSB e MSB juntos
	write_reg(addr_gyro_acc, CTRL3_C, 0x44);

	// Habilita bussola
	write_reg(addr_comp, LIS3MDL_CTRL_REG2, 0x00);
	write_reg(addr_comp, LIS3MDL_CTRL_REG1, 0xFC);
	write_reg(addr_comp, LIS3MDL_CTRL_REG4, 0x0C);
	write_reg(addr_comp, LIS3MDL_CTRL_REG3, 0x00);

	Thread::wait(100);
}

#define offset_gyro 0.035373f
#define new_offset 0.0435021f
#define gyro_off_a 0.0266207f
imu_data IMU::read_imu_data(bool use_mag) {
	imu_data data{};
	int16_t gyro_data, mag_data[2];

	read_reg(addr_gyro_acc, OUTZ_L_G, (char *) &gyro_data, 2);
//	data.gyro_z_rate = ((gyro_data * MAX_GYRO/INT16_MAX) + gyro_off_a)*(237611.9f/205838.1f) - 0.016999742f;
	data.gyro_z_rate = (gyro_data * MAX_GYRO/INT16_MAX)*(237611.9f/205838.1f) + 0.02063384f;

	if(use_mag) {
		read_reg(addr_comp, LIS3MDL_OUT_X_L, (char *) &mag_data, 4);
//		data.mag_x = (mag_data[0] * MAX_MAG / INT16_MAX + MAG_X_OFF) / MAG_X_MAX;
//		data.mag_y = (mag_data[1] * MAX_MAG / INT16_MAX + MAG_Y_OFF) / MAG_Y_MAX;
		data.mag_x = (mag_data[0] * MAX_MAG / INT16_MAX);
		data.mag_y = (mag_data[1] * MAX_MAG / INT16_MAX);
//		data.mag_z_theta = std::atan2(-data.mag_y, data.mag_x);
	}

	return data;
}

float IMU::read_gyro() {
	int16_t gyro_data;
	read_reg(addr_gyro_acc, OUTZ_L_G, (char *) &gyro_data, 2);
//	return (gyro_data * (MAX_GYRO/INT16_MAX) * (346722.0f/298599.0f));
	return -gyro_data * (MAX_GYRO/INT16_MAX) * (0.006934109f / 0.006106417f);
//	return gyro_data * (MAX_GYRO/INT16_MAX);
}

float IMU::read_mag() {
	int16_t mag_data[2];
	read_reg(addr_comp, LIS3MDL_OUT_X_L, (char *) &mag_data, 4);
//	float mag_x = (mag_data[0] * MAX_MAG / INT16_MAX);
//	float mag_y = (mag_data[1] * MAX_MAG / INT16_MAX);
	float mag_x = (mag_data[0] * MAX_MAG / INT16_MAX + MAG_X_OFF_b) / MAG_X_MAX_b;
	float mag_y = (mag_data[1] * MAX_MAG / INT16_MAX + MAG_Y_OFF_b) / MAG_Y_MAX_b;
	return std::atan2(-mag_y, mag_x);
}

mag_components IMU::read_mag_components() {
	int16_t mag_data[2];
	read_reg(addr_comp, LIS3MDL_OUT_X_L, (char *) &mag_data, 4);
	float mag_x = (mag_data[0] * MAX_MAG / INT16_MAX + MAG_X_OFF_b) / MAG_X_MAX_b;
	float mag_y = (mag_data[1] * MAX_MAG / INT16_MAX + MAG_Y_OFF_b) / MAG_Y_MAX_b;
//	float mag_x = (mag_data[0] * MAX_MAG / INT16_MAX);
//	float mag_y = (mag_data[1] * MAX_MAG / INT16_MAX);
	return {mag_x, mag_y};
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas do acelerometro
void IMU::read_acc_all(int16_t *data) {
	read_reg(addr_gyro_acc, OUTX_L_XL, (char *) data, 6);
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas do giroscopio
void IMU::read_gyro_all(int16_t *data) {
	read_reg(addr_gyro_acc, OUTX_L_G, (char *) data, 6);
}

// Recebe um vetor e preenche as 3 primeiras posicoes com as medidas da bussola
void IMU::read_comp_all(int16_t *data) {
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

