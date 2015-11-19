#ifndef SERIAL_H
#define SERIAL_H

//#define USART_BAUDRATE 9600
#define USART_BAUDRATE 57600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) -1)

#include <stdio.h>
// serial send data handler
int USART0SendByte(char u8Data, FILE *stream);
// serial recieve data handler
int USART0ReceiveByte(FILE *stream);
// serial initialize
void USART0Init(void);

#endif
