#include "AdpsSensor.h"

AdpsSensor::AdpsSensor(PinName sda, PinName scl) : i2c(sda, scl) {
	i2c.frequency(400000);
	write_reg(AdpsReg::CONTROL, 3 << 2 | 3);
//	write_reg(AdpsReg::ATIME, 200);
	write_reg(AdpsReg::CONFIG2, 3 << 4);
//	write_reg(AdpsReg::CONFIG2, 0);
	write_reg(AdpsReg::GCONF2, 3 << 5);
//	write_reg(AdpsReg::ENABLE, 1 | 1 << 2 | 1 << 1 | 1 << 6);
	write_reg(AdpsReg::ENABLE, 1 | 1 << 2 | 1 << 6);
//	write_reg(AdpsReg::ENABLE, 1 | 1 << 6);
//	write_reg(AdpsReg::ENABLE, 1 | 1 << 2 | 1 << 1);
	wait_ms(10);
}

uint8_t AdpsSensor::read_proximity() {
	uint8_t data = 0;
	read_reg(AdpsReg::PDATA, &data, 1);
	return data;
}

Location AdpsSensor::read_location() {
	Location location{};
	read_reg(AdpsReg::GFIFO_U, &location.up, 1);
	read_reg(AdpsReg::GFIFO_D, &location.down, 1);
	read_reg(AdpsReg::GFIFO_L, &location.left, 1);
	read_reg(AdpsReg::GFIFO_R, &location.right, 1);
	return location;
}

ColorRGBC AdpsSensor::read_color() {
	ColorRGBC color{};
	read_reg(AdpsReg::RDATAL, (uint8_t *) &color.r, 2);
	read_reg(AdpsReg::GDATAL, (uint8_t *) &color.g, 2);
	read_reg(AdpsReg::BDATAL, (uint8_t *) &color.b, 2);
	read_reg(AdpsReg::CDATAL, (uint8_t *) &color.c, 2);
	return color;
}

void AdpsSensor::read_reg(AdpsReg reg, uint8_t* data, int num_bytes) {
	i2c.write(addr, (char *) &reg, 1, true);
	i2c.read(addr, (char *) data, num_bytes);
}

void AdpsSensor::write_reg(AdpsReg reg, uint8_t data) {
	uint8_t cmd[2];
	cmd[0] = (uint8_t) reg;
	cmd[1] = data;
	i2c.write(addr, (char *) cmd, 2);
}
