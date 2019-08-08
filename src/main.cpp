#include "mbed.h"
#include "nRF24L01P.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
nRF24L01P nrf(p5, p6, p7, p8, p9, p10);    // mosi, miso, sck, csn, ce, irq
Serial pc(USBTX, USBRX); // tx, rx

int main() {
	constexpr int TRANSFER_SIZE = 12;
	pc.baud(921600);
	nrf.powerUp();

	wait(5);
	nrf.setRxAddress(0xE7E7E7E7E7, 3, 0);
	nrf.setTxAddress(0xE727E7E7E6, 3);
	nrf.setAirDataRate(2000);
	wait(2);

	nrf.setTransferSize(TRANSFER_SIZE);

	nrf.setReceiveMode();
	nrf.enable();
	char from_nrf[TRANSFER_SIZE], from_pc[TRANSFER_SIZE];
	pc.printf("it works!\r\n");

	while (true) {
		while (!nrf.readable(0) && !pc.readable()) Thread::wait(1);
		if (pc.readable()) {
			pc.scanf("%s", from_pc);
//			pc.printf("%s", from_pc);
			nrf.write(0, from_pc, TRANSFER_SIZE);
//			pc.printf("%s\r\n", from_pc);
		} else {
			nrf.read(0, from_nrf, TRANSFER_SIZE);
			pc.printf("%s", from_nrf);
			memset(&from_nrf, 0, TRANSFER_SIZE);
		}
	}
}

#pragma clang diagnostic pop