#include <stdint.h>
#include "serial.h"
#include <avr/io.h>

// serial send data handler
int USART0SendByte(char u8Data, FILE *stream) {
  if(u8Data == '\n') {
    USART0SendByte('\r', 0);
  }
  // wait
  while( !(UCSR0A & _BV(UDRE0) ) );
  // transmit
  UDR0 = u8Data;
  return 0;
}

// serial recieve data handler
int USART0ReceiveByte(FILE *stream) {
  uint8_t u8Data;
  // wait
  while( !(UCSR0A & _BV(RXC0) ) );
  u8Data = UDR0;
  // echo
  USART0SendByte(u8Data, stream);
  return u8Data;
}

// stream fle to write to
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, USART0ReceiveByte, _FDEV_SETUP_RW);
//FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

// serial initialize
void USART0Init(void) {
  // set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  // set frame to 8 data bits no parity
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
  // enable transmission and reception
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);

  // setup stream file to standard io to use printf
  stdin = stdout = &usart0_str;
}
