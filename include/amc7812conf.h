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

#define AMC7812_TIMEOUT_CONV_CYCLS 16000 // 1 ms at 16 MHz clock, ignoring overhead

// AMC7812 SPI port
#define AMC7812_SPI_PORT  PORTB
#define AMC7812_SPI_DDR   DDRB
#define AMC7812_SPI_SCLK  5
#define AMC7812_SPI_MISO  4
#define AMC7812_SPI_MOSI  3
// EVEN IF YOU USE A DIFFERENT CS PIN B2 (atmega 328p, 10 on UNO) MUST BE AN OUTPUT, 
// see atmeg328p man
#define AMC7812_SPI_SS    2

// AMC7812 control 

// REQUIRED: actual chip select pin, change here
#define AMC7812_CS_PORT   PORTB
#define AMC7812_CS_DDR    DDRB
#define AMC7812_CS_PIN    2     // UNO pin 10,
// REQUIRED:reset pin
#define AMC7812_RST_PORT  PORTB
#define AMC7812_RST_DDR   DDRB
#define AMC7812_RST_PIN   1    // UNO pin 9

// REQUIRED: data available flag (comment out if not connected)
#define AMC7812_DAV_PORT  PORTD
#define AMC7812_DAV_DDR   DDRD
#define AMC7812_DAV_PIN   2    // UNO pin ?
#define AMC7812_DAV_INT   INT0 // comment out if not connected to interrupt pin

// OPTIONAL: conversion trigger (comment out if not connected)
#define AMC7812_CNVT_PORT PORTB
#define AMC7812_CNVT_DDR  DDRB
#define AMC7812_CVNT_PIN  0    // UNO pin 8

#endif
