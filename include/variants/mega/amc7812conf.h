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
#define AMC7812_CS_PIN    7     // mega pin 42
#define AMC7812_CS_ARDUINO_PIN 42

// REQUIRED:reset pin
#define AMC7812_RST_PORT  PORTL
#define AMC7812_RST_DDR   DDRL
#define AMC7812_RST_PIN   1    // mega pin 48
#define AMC7812_RST_ARDUINO_PIN 48

// OPTIONAL: data available flag (comment out if not connected)
#define AMC7812_DAV_PORT  PORTL
#define AMC7812_DAV_DDR   DDRL
#define AMC7812_DAV_PIN   3    // mega pin 46
//#define AMC7812_DAV_INT   INT0 // comment out if not connected to interrupt pin

// OPTIONAL: conversion trigger (comment out if not connected)
#define AMC7812_CNVT_PORT PORTL
#define AMC7812_CNVT_DDR  DDRL
#define AMC7812_CNVT_PIN  6    // mega pin 43

// OPTIONAL: alarm (comment out if not connected)
#define AMC7812_ALARM_PORT  PORTL
#define AMC7812_ALARM_DDR   DDRL
#define AMC7812_ALARM_PIN   2    // mega pin 47
//#define AMC7812_ALARM_INT   INT0 // comment out if not connected to interrupt pin

// OPTIONAL: dac clear 0 (comment out if not connected)
#define AMC7812_DAC_CLR_0_PORT PORTL
#define AMC7812_DAC_CLR_0_DDR  DDRL
#define AMC7812_DAC_CLR_0_PIN  4    // mega pin 45

// OPTIONAL: dac clear 1 (comment out if not connected)
#define AMC7812_DAC_CLR_1_PORT PORTL
#define AMC7812_DAC_CLR_1_DDR  DDRL
#define AMC7812_DAC_CLR_1_PIN  5    // mega pin 44


// FRONT PANEL PIN MAPPINGS (use enumerated values in frontpanel.h in code)
#define AMC7812_IDLE_LED_ARDUINO 13
#define AMC7812_COMM_LED_ARDUINO 12
#define AMC7812_SCL_LED_ARDUINO 21
#define AMC7812_SDA_LED_ARDUINO 20

// avr pins
#define AMC7812_IDLE_LED_DDR DDRB
#define AMC7812_IDLE_LED_PORT PORTB
#define AMC7812_IDLE_LED_PIN 7

#define AMC7812_COMM_LED_DDR DDRB
#define AMC7812_COMM_LED_PORT PORTB
#define AMC7812_COMM_LED_PIN 6


// FRONT PANEL DIO Arduino
#define AMC7812_DIO0_ARDUINO 8
#define AMC7812_DIO1_ARDUINO 9
#define AMC7812_DIO2_ARDUINO 10
#define AMC7812_DIO3_ARDUINO 11

// FRONT PANEL DIO avr
#define AMC7812_DIOE_DDR DDRH
#define AMC7812_DIO0_PORT PORTH
#define AMC7812_DIO0_PIN 5

#define AMC7812_DIOE_DDR DDRH
#define AMC7812_DIO1_PORT PORTH
#define AMC7812_DIO1_PIN 6

#define AMC7812_DIOE_DDR DDRB
#define AMC7812_DIO2_PORT PORTB
#define AMC7812_DIO2_PIN 4

#define AMC7812_DIOE_DDR DDRB
#define AMC7812_DIO2_PORT PORTB
#define AMC7812_DIO3_PIN 5

#endif
