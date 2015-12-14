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
// uC is critcal path in data stream, so speed > memory

#include "avr/io.h"
#include <util/delay.h>

#include "amc7812.h"
#include "amc7812conf.h"
#include "amc7812err.h"

//#include <Arduino.h> // for deugging only (serial comms)

//AMC7812Class AMC7812;

//=============================================================================
//=============================================================================
//  UTILITY FUNCTIONS
//=============================================================================
//=============================================================================

//! Low-level frame transfer to chip
/*!
  * \param addr is the register address to be read from or written to
  * \param val is the value to be written to the register, value is not read for a read operation
  * \return returns last 2-bytes as response from previous frame
  *
  * Low-level SPI frame transfer protocol.
  * A frame is 3-bytes in the format: [ register address, value[15:8], value[7:0] ].
  *
  * _Note_: AMC7812_CS_PIN amd AMC7812_CS_PORT must be set correctly in amc7812conf.h
  */
uint16_t AMC7812Class::transfer ( uint8_t cmd, uint16_t data ){
  // avr is little endian
  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
  in.val = data;

  // assert CS low
  AMC7812_CS_PORT &= ~(1<<AMC7812_CS_PIN);

  SPDR = cmd;
  /*  from arduino 
    * The following NOP introduces a small delay that can prevent the wait
    * loop form iterating when running at the maximum speed. This gives
    * about 10% more speed, even if it seems counter-intuitive. At lower
    * speeds it is unnoticed.
    */
  asm volatile("nop");
  while (!(SPSR & _BV(SPIF))) ; // wait
  // send second byte
  SPDR = in.msb;
  asm volatile("nop"); // See transfer(uint8_t) function
  while (!(SPSR & _BV(SPIF))) ;
  out.msb = SPDR; // save response
  // send third byte
  SPDR = in.lsb;
  asm volatile("nop");
  while (!(SPSR & _BV(SPIF))) ;
  out.lsb = SPDR; // save response

  // release CS high
  AMC7812_CS_PORT |= (1<<AMC7812_CS_PIN);

  return out.val;
}

//=============================================================================
//=============================================================================
//  PUBLIC FUNCTIONS
//=============================================================================
//=============================================================================

//! prepare device for SPI communication
/*!
  * Tasks to be performed:
  * - Reset device
  * - Setup uC SPI interface
  * - Verify device ID
  * - Configure DACs
  * - Configure ADCs
  * - Trigger ADC read cycle
  */
uint8_t AMC7812Class::begin(){
  // in case SS is not the same as AMC7812_CS_PIN/PORT 
  AMC7812_CS_PORT |= (1<<AMC7812_CS_PIN); // turn pin high first
  AMC7812_CS_DDR  |= (1<<AMC7812_CS_PIN);   // then set as output
//#if ( (AMC7812_CS_PORT != AMC7812_SS_PORT) & (AMC7812_CS_PIN != AMC7812_SPI_SS) )
  // SS _HAS_ to be an output, else chip will enter slave mode on low input
  AMC7812_SPI_PORT |= (1<<AMC7812_SPI_SS);
//#endif
  // set up SCLK, MOSI, MISO, and SS as outputs, and enambles SPI as master
  // set MOSI & SCK & CS as output
  AMC7812_SPI_DDR = _BV(AMC7812_SPI_SCLK) | _BV(AMC7812_SPI_MOSI) | _BV(AMC7812_SPI_SS);
  // setup spi status register, SPI mode 1
  // prescalar 2, f/2
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA); // 4x prescalar
  SPSR = (1<<SPI2X); // double time
 
  
#ifdef AMC7812_CNVT_PIN // setup !convert pin if attached
  #pragma message "External Triggering is enabled for AMC7812"
  AMC7812_CNVT_DDR |= (1<<AMC7812_CNVT_PIN);
#else
  #pragma message "External Triggering is disabled for AMC7812"
#endif

#ifdef AMC7812_DAV_PIN // setup data available pin if attached
  #pragma message "External Data Available pin is enabled for AMC7812"
  AMC7812_DAV_DDR &= ~(1<<AMC7812_DAV_PIN);  // data available pin is input
#else
  #pragma message "External Data Available pin is disabled for AMC7812"
#endif

  // reset device
  // reset pulse width - 20 ns (50 MHz), pg. 6
  // time until normal operation - 250 seconds, pg. 6
  // TODO: add normal operation flag
  AMC7812_RST_DDR  |= (1<<AMC7812_RST_PIN);
  AMC7812_RST_PORT &= ~(1<<AMC7812_RST_PIN);
  AMC7812_RST_PORT |= (1<<AMC7812_RST_PIN);

  _delay_us(100);  // necessary delay determined experimentally

  // read the device ID register and verify that the expected value matches
  Read( AMC7812_DEV_ID );
  // dummy read to get the answer
  uint16_t response = Read( 0x00 );
  if( response != 0x1220 ){
    return AMC7812_DEV_ID_ERR;
  }

  // set dac gain to low range by default
  uint16_t gain_sp = 0x0000;
  WriteDACGains( gain_sp );
  ReadDACGains();
  // check that dac gain has been set low
  if( dac_gain != gain_sp ){
    return AMC7812_WRITE_ERR;
  }

  // turn on DAC output on all channels
  uint16_t pwr_dwn_reg = 0x7FFE;
  Write( AMC7812_POWER_DOWN, pwr_dwn_reg );
  Read( AMC7812_POWER_DOWN );
  response = Read( 0x00 );
  // check that dac gain has been set low
  if( response != pwr_dwn_reg ){
    return AMC7812_WRITE_ERR;
  }

  // set the ADC ref as internal, and continuous read mode
  uint16_t amc_config = (1<<AMC7812_CMODE)|(1<<AMC7812_ADC_REF);
  WriteAMCConfig( 0, amc_config );
  // turn on the adcs
  EnableADCs();
  TriggerADCsInternal();
  
  return 0; // no errors in initalization
}

//==============================================================================
// ADC FUNCTIONS
//==============================================================================

//! Batch read operation, faster operatoin is possible with assumptions
/*! 
  * Function is provided to conveniently measure the enabled ADC channels.
  * If continuous mode, write conversion flag to status register and read ADCs.
  * If triggered mode, send conversion trigger and read activated ADCs.
  * Values are stored in `adc_vals` member, retrieve with `GetADCReadings()`. 
  */
uint8_t AMC7812Class::ReadADCs(){
  // if convert pin is connected, then use it for triggerring, 
  // otherwise use the internal trigger
  if ( !(amc_conf[0] & (1<<AMC7812_CMODE)) ){
#ifdef AMC7812_CNVT_PIN
    TriggerADCsExternal();
#else
    TriggerADCsInternal();
#endif
    // wait for data available, polling
    // TODO: timeout cycles should have prefactor based on conversion speed 
    // setting and clock frequency
    for(uint16_t i=0; i<AMC7812_TIMEOUT_CONV_CYCLS; i++){
      if ( !(AMC7812_DAV_PORT & (1<<AMC7812_DAV_PIN)) ){ break; }
    }
    return AMC7812_TIMEOUT_ERR;
  }

  int8_t last_valid = -1;
  for(uint8_t i=0; i<AMC7812_ADC_CNT; i++){
    if ( adc_status & (1<<i) ){
      uint16_t reading = ReadADC(i);
      if ( last_valid >= 0 ){
        adc_vals[last_valid] = reading;
      }
      last_valid = i;
    } else {
      adc_vals[i] = 0;
    }
  }
  // dummy read, if statement not necessary if at least one ADC is enabled
  if ( last_valid >= 0 ){
    adc_vals[last_valid] = ReadADC(0);
  }
  
  return 0;
}

//! Enable ADCn
/*!
  * \param n is an integer between 0 and 15 (a check is performed) to enable ADCn
  * \return returned value is the response for the previous frame
  *
  * Enabled ADCs are recorded everytime a reading is triggered.
  * If faster reads are required, disable unused channels to decrease cycle time
  * and latency.
  * ADC enabled/disabled status is stored bitwise in `adc_status` a high bit
  * signifies an enabled channel.
  * Use `GetADCStatus()` to retrieve `adc_status` member value.
  */
uint16_t AMC7812Class::EnableADC( uint8_t n ){
  uint16_t setval = 0; // if n out-of-range setval=0 and nothing happens
  // stupid register struture makes this crappy, pg 71 
  if( n < 13 ){
    setval = (1<<(12-n));
    if ( n < 4 ){
      setval = setval << 1;
      if ( n < 2 ){
        setval = setval << 1;
      }
    }
  } else {
    if ( n < AMC7812_ADC_CNT ){
      setval = 1<<(27-n);
    }
  }
  adc_status |= setval;  // save new status

  // adc channels > 12 are stored in ADC_1_CONF, one address above
  return Write( AMC7812_ADC_0_CONF + n/13, adc_status ); 
}

//! Enable All ADCs
/*!
  * \return returned value is the response for the previous frame
  *
  * _Note_ this operation requires two frames to set all values.
  * The returned value is from the command preceeding both.
  *
  * Enabled ADCs are recorded everytime a reading is triggered.
  * If faster reads are required, disable unused channels to decrease cycle time
  * and latency.
  * ADC enabled/disabled status is stored bitwise in `adc_status` a high bit
  * signifies an enabled channel.
  * Use `GetADCStatus()` to retrieve `adc_status` member value.
  */
uint16_t AMC7812Class::EnableADCs(){
  // first command
  uint16_t response = Write( AMC7812_ADC_0_CONF, 0x6DFF );
  // second command
  Write( AMC7812_ADC_1_CONF, 0x7000 );
  // save adc status to class field
  adc_status = 0xFFFF;
  return response;
}

//! Disable ADCn
/*!
  * \param n is an integer between 0 and 15 (a check is performed) to disable ADCn
  * \return returned value is the response for the previous frame
  *
  * Enabled ADCs are recorded everytime a reading is triggered.
  * If faster reads are required, disable unused channels to decrease cycle time
  * and latency.
  * ADC enabled/disabled status is stored bitwise in `adc_status` a high bit
  * signifies an enabled channel.
  * Use `GetADCStatus()` to retrieve `adc_status` member value.
  */
uint16_t AMC7812Class::DisableADC( uint8_t n ){
  uint16_t setval = 0; // if n out-of-range setval=0 and nothing happens
  // stupid register struture makes this crappy, pg 71 
  if( n < 13 ){
    setval = (1<<(12-n));
    if ( n < 4 ){
      setval = setval << 1;
      if ( n < 2 ){
        setval = setval << 1;
      }
    }
  } else {
    if ( n < AMC7812_ADC_CNT ){
      setval = 1<<(27-n);
    }
  }
  adc_status &= ~setval; // reference old status

  // adc channels > 12 are stored in ADC_1_CONF, one address above
  return Write( AMC7812_ADC_0_CONF + n/13, adc_status ); 
}


//! Disable All ADCs
/*!
  * \return returned value is the response for the previous frame
  *
  * _Note_ this operation requires two frames to set all values.
  * The returned value is from the command preceeding both.
  *
  * Enabled ADCs are recorded everytime a reading is triggered.
  * If faster reads are required, disable unused channels to decrease cycle time
  * and latency.
  * ADC enabled/disabled status is stored bitwise in `adc_status` a high bit
  * signifies an enabled channel.
  * Use `GetADCStatus()` to retrieve `adc_status` member value.
  */
uint16_t AMC7812Class::DisableADCs(){
  // first command
  uint16_t response = Write( AMC7812_ADC_0_CONF, 0x0000 );
  // second command
  Write( AMC7812_ADC_1_CONF, 0x0000 );
  // save adc status to class field
  adc_status = 0x0000;
  for(uint8_t i=0; i<AMC7812_ADC_CNT; i++){
    adc_vals[i]=0;
  }
  return response;
}

//==============================================================================
// DAC FUNCTIONS
//==============================================================================

//==============================================================================
// CONFIGURATION REGISTERS FUNCTIONS
//==============================================================================

//! Write to one of the chip main configuration registers
/*!
  *  \param n is the AMC configuraion regster to be address, 0 or 1 (checked)
  *  \param config is the register configuration value (masked)
  *  \return 0 if no error, AMC_PARAM_OOR_ERR if n is out of range
  *  
  *  Flags and internal trigger bits (ICONV & ILDAC) cannot be set through this
  *  function.
  *
  *  See page 66.
  */
uint8_t AMC7812Class::WriteAMCConfig( uint8_t n, uint16_t config ){
  // only save (R/W) values not flags or trigger values (ICONV & ILDAC)
  switch(n){
    case 0:
      config = 0x2600 & config;
      break;
    case 1:
      config = 0x03F8 & config;
      break;
    default:
      return AMC7812_PARAM_OOR_ERR;
  }
  amc_conf[n] = config;
  Write( AMC7812_WRITE_MASK|(AMC7812_AMC_CONF_0 + n), config);
  return 0;
}
