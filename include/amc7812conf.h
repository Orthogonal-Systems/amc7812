/*****************************************************************************
*
* Title  : Texas Instruments AMC7812 Integrated ADC & DAC Driver
* Author : Matthew Ebert (c)2015
* Copyright: GPL V3
*
* This driver provides initialization and transmit/receive
* functions for the TI AMC7812 integrated ADC and DAC.
* Only SPI standalone mode is supported.
*
*****************************************************************************/

#ifndef AMC7812CONF_H
#define AMC7812CONF_H

// AMC7812 SPI port
#define AMC7812_SPI_PORT  PORTB
#define AMC7812_SPI_DDR   DDRB
#define AMC7812_SPI_SCLK  5
#define AMC7812_SPI_MISO  4
#define AMC7812_SPI_MOSI  3

// AMC7812 control 
#define AMC7812_CS_PORT   PORTB
#define AMC7812_CS_DDR    DDRB
#define AMC7812_CS_PIN    2     // UNO pin 10

#define AMC7812_RST_PORT   PORTB
#define AMC7812_RST_DDR    DDRB
#define AMC7812_RST_PIN    1    // UNO pin 9

#endif
