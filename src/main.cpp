#include "mbed.h"
#include "nRF24L01P.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
Serial pc(USBTX, USBRX); // tx, rx
nRF24L01P nrf(p5, p6, p7, p8, p9, p10);    // mosi, miso, sck, csn, ce, irq

int main() {
	constexpr int TRANSFER_SIZE = 12;
	pc.baud(115200);

	nrf.powerUp();

	wait(5);
	nrf.setTransferSize(4, 0);
	nrf.setRfOutputPower();
	nrf.setTxAddress(0xE7E7E7E7E7, 3);
	nrf.setRxAddress(0xE727E7E7E6, 3, 0);
	nrf.setAirDataRate(2000);

//	nrf.enableAutoAcknowledge();
//	nrf.enableAutoRetransmit(250, 2);
	wait(2);
	// Display the (default) setup of the nRF24L01+ chip
	pc.printf("nRF24L01+ Frequency    : %d MHz\r\n", nrf.getRfFrequency());
	pc.printf("nRF24L01+ Output power : %d dBm\r\n", nrf.getRfOutputPower());
	pc.printf("nRF24L01+ Data Rate    : %d kbps\r\n", nrf.getAirDataRate());
	pc.printf("nRF24L01+ TX Address   : %x\r\n", static_cast<unsigned int>(nrf.getTxAddress()));
	pc.printf("nRF24L01+ RX Address   : %x\r\n", static_cast<unsigned int>(nrf.getRxAddress()));
//	pc.printf("nRF24L01+ TX Address   : 0x%010llX\r\n", nrf.getTxAddress());
//	pc.printf("nRF24L01+ RX Address   : 0x%010llX\r\n", nrf.getRxAddress());

	nrf.setTransferSize(TRANSFER_SIZE);

	nrf.setReceiveMode();
	nrf.enable();

	int sent = 0;
	int lost = 0;
	int wrong = 0;
	int received = 0;

	Timer t;
	t.start();
	Timer t2;
	t2.start();
	while (true) {
		auto elapsed = -1000000;
		t2.reset();
		nrf.write(0, (char *) &sent, 12);
		t.reset();
		while (!nrf.readable() && t.read() < 0.066);
		if (nrf.readable()) {
			nrf.read(0, (char *) &received, 12);
			elapsed = t2.read_us();
			if (received != sent) {
				wrong++;
			}
		} else {
			lost++;
		}
		pc.printf("lost: %lf (%i), wrong: %lf, sent: %i, received: %i, time: %i\n",
				(lost * 100.0) / (sent + 1), lost,
				  (wrong * 100.0) / (sent + 1), sent, received, elapsed);
		sent++;
	}
}

#pragma clang diagnostic pop