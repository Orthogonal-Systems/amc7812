/*****************************************************************************
*
* Title  : Front Panel Driver for MegaDAQ Board
* Author : Matthew Ebert (c)2016
* Copyright: GPL V3
*
* This driver provides functions for controlling the LEDs on
* the front panel of the MegaDAQ.
* The LED driver is a TPIC2810, I2C slave address is 96 (group=0xC, device=0x0)
*
*****************************************************************************/

#ifndef FRONTPANEL_H
#define FRONTPANEL_H
#pragma message "using frontpanel file found at path: " __FILE__

// functions
#include <stdint.h>

// digital IO enum
#define FRONTPANEL_DIO0  0x01
#define FRONTPANEL_DIO1  0x02
#define FRONTPANEL_DIO2  0x04
#define FRONTPANEL_DIO3  0x08

// TPIC2810 ADDRESSES
#define FRONTPANEL_MCU_STAT_LED_ADDR 5
#define FRONTPANEL_AMC_STAT_LED_ADDR 4
#define FRONTPANEL_USER1_LED_ADDR 6
#define FRONTPANEL_USER2_LED_ADDR 7
#define FRONTPANEL_ERR1_LED_ADDR 0
#define FRONTPANEL_ERR2_LED_ADDR 1
#define FRONTPANEL_ERR3_LED_ADDR 3
#define FRONTPANEL_ERR4_LED_ADDR 2

#define FRONTPANEL_TPIC2810_ADDR 96
// subaddress command to read new value and update
#define FRONTPANEL_TPIC2810_UPDATE 0x44 

// FRONT PANEL DIO AND LEDS, ENUMERATIONS
enum LED {
  // TPIC2810 LEDs
  MCU_STAT_LED, //!< microcontroller status (power mostly)
  AMC_STAT_LED, //!< MegaDAQ status
  USER1_LED,    //!< User defined status
  USER2_LED,    //!< User defined status
  ERR1_LED,     //!< Error LED
  ERR2_LED,     //!< Error LED
  ERR3_LED,     //!< Error LED
  ERR4_LED,     //!< Error LED
  // MCU LEDs (faster control)
  IDLE_LED,     //!< Waiting For Trigger LED
  COMM_LED      //!< Communication LED
};

//! Update all leds
uint8_t set_tpic2810_leds( uint8_t leds );

//! Update led at address addr to value (boolean)
uint8_t set_tpic2810_led( uint8_t addr, uint8_t value );

//! Set the status of a single LED on the front panel
/*!
 * \param led is the enumerated pin number to be set, use definitions in this file.
 * \param value is the state of the pin 0 is off, >0 is on.
 */
void frontpanel_set_led( LED led, uint8_t value );

//! Setup pins used by front panel and initialize i2c interface
void frontpanel_setup();

#endif
