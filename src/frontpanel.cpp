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

// functions
#include <avr/io.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h> // I2C
#include "frontpanel.h"
#include "amc7812conf.h"

/*
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
*/

#define unset_dout(port,mask) ((port) &= ~(mask)) 
#define set_dout(port,mask) ((port) |= (mask)) 

uint8_t fp_status;  //!< Current LED Control Values for tpic2810

//! Update all leds
uint8_t set_tpic2810_leds( uint8_t leds ){
  Wire.beginTransmission(FRONTPANEL_TPIC2810_ADDR);
  Wire.write(FRONTPANEL_TPIC2810_UPDATE);
  Wire.write(leds);
  uint8_t ret = Wire.endTransmission();
  if(ret == 0){
    fp_status = leds;
  }
  return ret;
}

//! Update led at address addr to value (boolean)
uint8_t set_tpic2810_led( uint8_t addr, uint8_t value ){
  uint8_t leds;
  if(value){
    leds = fp_status | 1 << addr;
  } else {
    leds = fp_status & ~(1 << addr);
  }
  return set_tpic2810_leds( leds );
}

//! Set the status of a single LED on the front panel
/*!
 * \param led is the enumerated pin number to be set, use definitions in this file.
 * \param value is the state of the pin 0 is off, >0 is on.
 */
void frontpanel_set_led( LED led, uint8_t value ){
  switch(led){
    case MCU_STAT_LED: set_tpic2810_led(FRONTPANEL_MCU_STAT_LED_ADDR, value);
                  break;
    case AMC_STAT_LED: set_tpic2810_led(FRONTPANEL_AMC_STAT_LED_ADDR, value);
                  break;
    case USER1_LED: set_tpic2810_led(FRONTPANEL_USER1_LED_ADDR, value);
                  break;
    case USER2_LED: set_tpic2810_led(FRONTPANEL_USER2_LED_ADDR, value);
                  break;
    case ERR1_LED : set_tpic2810_led(FRONTPANEL_ERR1_LED_ADDR, value);
                  break;
    case ERR2_LED : set_tpic2810_led(FRONTPANEL_ERR2_LED_ADDR, value);
                  break;
    case ERR3_LED : set_tpic2810_led(FRONTPANEL_ERR3_LED_ADDR, value);
                  break;
    case ERR4_LED : set_tpic2810_led(FRONTPANEL_ERR4_LED_ADDR, value);
                  break;

    // MCU leds are active low
    case IDLE_LED : if(value){
                    AMC7812_IDLE_LED_PORT &= ~(1<<AMC7812_IDLE_LED_PIN); 
                  } else {
                    AMC7812_IDLE_LED_PORT |= (1<<AMC7812_IDLE_LED_PIN); 
                  }
                  break;
    case COMM_LED : if(value){
                    AMC7812_COMM_LED_PORT &= ~(1<<AMC7812_COMM_LED_PIN); 
                  } else {
                    AMC7812_COMM_LED_PORT |= (1<<AMC7812_COMM_LED_PIN); 
                  }
                  break;
    default     : Serial.println("shit");
  }
}

void flash(LED led){
  frontpanel_set_led( led, 1 );
  delay(150);
  frontpanel_set_led( led, 0 );
}

void frontpanel_setup(){
  Wire.begin();

  // set leds as outputs
  AMC7812_IDLE_LED_PORT |= (1<<AMC7812_IDLE_LED_PIN); 
  AMC7812_COMM_LED_PORT |= (1<<AMC7812_COMM_LED_PIN); 
  AMC7812_IDLE_LED_DDR |= (1<<AMC7812_IDLE_LED_PIN); 
  AMC7812_COMM_LED_DDR |= (1<<AMC7812_COMM_LED_PIN); 
  // turn on all leds and flash twice
  for( uint8_t i=0; i<2; i++ ){
    delay(500);
    AMC7812_IDLE_LED_PORT &= ~(1<<AMC7812_IDLE_LED_PIN); 
    AMC7812_COMM_LED_PORT &= ~(1<<AMC7812_COMM_LED_PIN); 
    set_tpic2810_leds( 0xff );
    delay(500);
    AMC7812_IDLE_LED_PORT |= (1<<AMC7812_IDLE_LED_PIN); 
    AMC7812_COMM_LED_PORT |= (1<<AMC7812_COMM_LED_PIN); 
    set_tpic2810_leds( 0x00 );
  }
  // cycle through all leds
  flash( MCU_STAT_LED );
  flash( AMC_STAT_LED );
  flash( IDLE_LED );
  flash( COMM_LED );
  flash( USER1_LED );
  flash( USER2_LED );
  flash( ERR1_LED );
  flash( ERR2_LED );
  flash( ERR3_LED );
  flash( ERR4_LED );

  // turn off all but the mcu power
  frontpanel_set_led( MCU_STAT_LED, 1 );
}
