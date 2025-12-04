#ifndef UART_H_
#define UART_H_

#include "common.h"

void uartInit(void);
void uartTransmitByte(uint8_t byte);
void uartTransmitDec(int16_t dec);
void uartTransmitStr(char *str);

#endif /* UART_H_ */
