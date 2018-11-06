#include "ApdsSensor.h"
#include "helper_functions.h"
#include "cmath"

ApdsSensor::ApdsSensor(PinName sda, PinName scl) : i2c(sda, scl) {
	i2c.frequency(400 * 1000);
	write_reg(ApdsReg::CONTROL, 3 << 2 | 3);
//	write_reg(ApdsReg::ATIME, 200);
//	write_reg(ApdsReg::CONFIG2, 3 << 4);
//	write_reg(ApdsReg::CONFIG2, 0);
	write_reg(ApdsReg::GCONF2, 3 << 5);
//	write_reg(ApdsReg::ENABLE, 1 | 1 << 2 | 1 << 1 | 1 << 6);
	write_reg(ApdsReg::ENABLE, 1 | 1 << 2 | 1 << 6);
//	write_reg(ApdsReg::ENABLE, 1 | 1 << 6);
//	write_reg(ApdsReg::ENABLE, 1 | 1 << 2 | 1 << 1);
	wait_ms(10);
}

ApdsObj ApdsSensor::get_obj() {
	auto location = read_location();
	auto prox = float(std::sqrt(std::pow(location.right, 2.0f) + std::pow(location.left, 2.0f)));
	auto theta = float(std::atan2(location.right, location.left)) - to_rads(45);
	return {prox, theta};
}

uint8_t ApdsSensor::read_proximity() {
	uint8_t data = 0;
	read_reg(ApdsReg::PDATA, &data, 1);
	return data;
}

Location ApdsSensor::read_location() {
	Location location{};
	read_reg(ApdsReg::GFIFO_U, &location.up, 1);
	read_reg(ApdsReg::GFIFO_D, &location.down, 1);
	read_reg(ApdsReg::GFIFO_L, &location.left, 1);
	read_reg(ApdsReg::GFIFO_R, &location.right, 1);
	return location;
}

ColorRGBC ApdsSensor::read_color() {
	ColorRGBC color{};
	read_reg(ApdsReg::RDATAL, (uint8_t *) &color.r, 2);
	read_reg(ApdsReg::GDATAL, (uint8_t *) &color.g, 2);
	read_reg(ApdsReg::BDATAL, (uint8_t *) &color.b, 2);
	read_reg(ApdsReg::CDATAL, (uint8_t *) &color.c, 2);
	return color;
}

void ApdsSensor::read_reg(ApdsReg reg, uint8_t *data, int num_bytes) {
	i2c.write(addr, (char *) &reg, 1, true);
	i2c.read(addr, (char *) data, num_bytes);
}

void ApdsSensor::write_reg(ApdsReg reg, uint8_t data) {
	uint8_t cmd[2];
	cmd[0] = (uint8_t) reg;
	cmd[1] = data;
	i2c.write(addr, (char *) cmd, 2);
}

