#include "mbed.h"
#include "nRF24L01P.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
nRF24L01P nrf(p5, p6, p7, p8, p9, p10);    // mosi, miso, sck, csn, ce, irq
Serial pc(USBTX, USBRX); // tx, rx

int main() {
	constexpr int TRANSFER_SIZE = 12;
	pc.baud(115200);
	nrf.powerUp();

	wait(5);
	nrf.setRxAddress(0xE7E7E7E7E7, 3, 0);
	nrf.setTxAddress(0xE727E7E7E6, 3);
	nrf.setAirDataRate(2000);
	wait(2);

	nrf.setTransferSize(TRANSFER_SIZE);

	nrf.setReceiveMode();
	nrf.enable();
	char data[12];

	while (true) {
		while (!nrf.readable(0)) Thread::wait(1);
		nrf.read(0, data, TRANSFER_SIZE);
		pc.printf("%s\r\n", data);
		memset(&data, 0, TRANSFER_SIZE);
	}
}

#pragma clang diagnostic pop