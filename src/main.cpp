#include "mbed.h"
#include "PIN_MAP.h"
#include "Controller.h"
#include <cmath>
#include <fstream>
#include <array>
#include <lib/nRF24L01p/nRF24L01P.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define PI 3.141592f

void led_write(std::array<DigitalOut, 4> &LEDs, uint8_t num) {
	LEDs[0] = ((num >> 0) & 1);
	LEDs[1] = ((num >> 1) & 1);
	LEDs[2] = ((num >> 2) & 1);
	LEDs[3] = ((num >> 3) & 1);
}

void bat_watcher(std::array<DigitalOut, 4> &LEDs, AnalogIn &battery_vin) {
	double vbat = battery_vin.read() * (3.3 * 1470 / 470);
	double threshold = (vbat - 6.6) / 1.4;

	if (threshold >= 0.75) led_write(LEDs, 0b1111);
	else if (threshold >= 0.5) led_write(LEDs, 0b0111);
	else if (threshold >= 0.25) led_write(LEDs, 0b0011);
	else led_write(LEDs, 0b0001);
}

constexpr int TRANSFER_SIZE = 12;

int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);
	nRF24L01P nrf(p5, p6, p7, p8, p9, p10);
	nrf.powerUp();
	wait(1);
	nrf.setTransferSize(4, 0);
	nrf.setRfOutputPower();
	nrf.setTxAddress(0xE7E7E7E7E7, 3);
	nrf.setRxAddress(0xE727E7E7E6, 3, 0);
	nrf.setAirDataRate(2000);
	nrf.setReceiveMode();
	nrf.enable();

	Controller controller;
	controller.set_pwm(controller.right_wheel, -0.5);

	while (true) {
		controller.update_wheel_velocity();
		nrf.write(0, (char *) &controller.right_wheel.velocity, 4);
		wait(1);
		bat_watcher(LEDs, battery_vin);
	}
}

#pragma clang diagnostic pop