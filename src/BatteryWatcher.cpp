#include <BatteryWatcher.h>

BatteryWatcher::BatteryWatcher() :
        battery_vin(ALL_CELLS),
        LEDs{DigitalOut(LED1), DigitalOut(LED2),
		     DigitalOut(LED3), DigitalOut(LED4)} {

}

void BatteryWatcher::led_write(uint8_t num) {
	LEDs[0] = ((num >> 0) & 1);
	LEDs[1] = ((num >> 1) & 1);
	LEDs[2] = ((num >> 2) & 1);
	LEDs[3] = ((num >> 3) & 1);
}

void BatteryWatcher::update_battery_leds() {
	double vbat = battery_vin.read() * (3.3 * 1470 / 470);
	double threshold = (vbat - 6.6) / 1.4;

	if (threshold >= 0.75) led_write(0b1111);
	else if (threshold >= 0.5) led_write(0b0111);
	else if (threshold >= 0.25) led_write(0b0011);
	else led_write(0b0001);
}