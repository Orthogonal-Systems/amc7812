/*****************************************************************************
*
* Title  : TPIC2810 Driver
* Author : Matthew Ebert (c)2018
* Copyright: GPL V3
*
* This driver provides functions for controlling the status of 
* the front panel of the MegaDAQ.
* The LED driver is a TPIC2810, I2C slave address is (0x60) 96+addr
* (group=0xC, device=A2,A1,A0)
*
*****************************************************************************/

#ifndef TPIC2810_H
#define TPIC2810_H
#pragma message "using tpic2810 file found at path: " __FILE__

#include <stdint.h>
#include <Wire.h> // I2C

#define TPIC2810_GROUP 96
// subaddress command to write new value and immediatly update
#define TPIC2810_IUPDATE 0x44 
// subaddress command to read/write new value to buffer but dont update
#define TPIC2810_RW 0x11 
// subaddress command to move buffer to output
#define TPIC2810_UPDATE 0x22 

//! Update all channels
uint8_t set_tpic2810_all( uint8_t addr, uint8_t out ){
  Wire.beginTransmission(TPIC2810_GROUP + (0x07&addr));
  Wire.write(TPIC2810_IUPDATE);
  Wire.write(out);
  return Wire.endTransmission();
}

//! Setup and initialize i2c interface
void setup_tpic2810(){
  Wire.begin();
}

#endif
