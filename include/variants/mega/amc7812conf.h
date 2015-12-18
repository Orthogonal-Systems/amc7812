/*******************************************************************************
*
* Title  : Texas Instruments AMC7812 Integrated ADC & DAC Driver
* Author : Matthew Ebert (c)2015
* Copyright: GPL V3
*
* This header defines default settings for connections for communication with
* an amc7812 ADC/DAC
*
* NOTE if you put this in the same directory as the executable you are
* compiling it overrides the default in [PROEJCT_DIR]/include/
*
*******************************************************************************/

#ifndef AMC7812CONF_H
#define AMC7812CONF_H

// I want a configuration file in the working directory to override the default
#pragma message "using configuration file found at path: " __FILE__

#define AMC7812_TIMEOUT_CONV_CYCLS 16000 // 1 ms at 16 MHz clock, ignoring overhead

// VOLTAGE REFERENCE
// store as string to send as in data packet since we dont want to do fp math
#define AMC7812_INT_AVREF "2.5"
#define AMC7812_AVREF AMC7812_INT_AVREF

// AMC7812 SPI port
#define AMC7812_SPI_PORT  PORTB
#define AMC7812_SPI_DDR   DDRB
#define AMC7812_SPI_SCLK  1   // mega pin 52
#define AMC7812_SPI_MISO  3   // mega pin 50
#define AMC7812_SPI_MOSI  2   // mega pin 51
// EVEN IF YOU USE A DIFFERENT CS PIN B2 (atmega 328p, 10 on UNO) MUST BE AN OUTPUT, 
// see atmeg328p man
#define AMC7812_HWCS_PIN  0   // mega pin 53

// AMC7812 control 

// REQUIRED: actual chip select pin, change here
#define AMC7812_CS_PORT   PORTL
#define AMC7812_CS_DDR    DDRL
#define AMC7812_CS_PIN    0     // mega pin 49
// REQUIRED:reset pin
#define AMC7812_RST_PORT  PORTL
#define AMC7812_RST_DDR   DDRL
#define AMC7812_RST_PIN   1    // mega pin 48

// OPTIONAL: data available flag (comment out if not connected)
#define AMC7812_DAV_PORT  PORTL
#define AMC7812_DAV_DDR   DDRL
#define AMC7812_DAV_PIN   2    // mega pin 47
//#define AMC7812_DAV_INT   INT0 // comment out if not connected to interrupt pin

// OPTIONAL: conversion trigger (comment out if not connected)
#define AMC7812_CNVT_PORT PORTL
#define AMC7812_CNVT_DDR  DDRL
#define AMC7812_CNVT_PIN  3    // mega pin 46

#endif
