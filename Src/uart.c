#include "../Inc/uart.h"

bool uartInitDone = false;

void uartTransmitByte(uint8_t byte) {

	USART2->DR = byte;
	while (!(USART2->SR & USART_SR_TXE)) {

		__NOP();
	}
}

void uartTransmitDec(int16_t dec) {

	if(dec == 0) {

		uartTransmitDec('0');
		return;
	}

	if (dec < 0) {

		dec = -dec;
		uartTransmitByte('-');
	} else {

		uartTransmitByte(' ');
	}

	uint8_t i;

	for (i = 1; i <= 5; i++) {

		if (digitExtractor(dec, i)) {

			break;
		} else {

			continue;
		}
	}

	for(uint8_t j = i; j <= 5; j++) {

		uartTransmitByte(digitExtractor(dec, j) + '0');
	}
}

void uartTransmitStr(char *str) {

	while(*str) {

		uartTransmitByte(*str++);
	}
}
