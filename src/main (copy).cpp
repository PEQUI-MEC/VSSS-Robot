#include "mbed.h"
#include "nRF24L01P.h"

nRF24L01P nrf(p5, p6, p7, p8, p9, p10);    // mosi, miso, sck, csn, ce, irq

int main() {
	constexpr int TRANSFER_SIZE = 12;

	nrf.powerUp();

	wait(5);
	nrf.setTransferSize(4, 0);
	nrf.setRxAddress(0xE7E7E7E7E7, 3, 0);
	nrf.setTxAddress(0xE727E7E7E6, 3);
	nrf.setAirDataRate(2000);
	wait(2);

	nrf.setTransferSize(TRANSFER_SIZE);

	nrf.setReceiveMode();
	nrf.enable();

	while (true) {
		int input;
		while (!nrf.readable(0));
		nrf.read(0, (char *) &input, 12);
		nrf.write(0, (char *) &input, 12);
	}
}