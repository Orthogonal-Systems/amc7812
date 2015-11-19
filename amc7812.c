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
// TODO: 4 MSBs hold errors on 12-bit errors

// uC is critcal path in data stream, so speed > memory

#include "avr/io.h"
#include <util/delay.h>

#include "amc7812.h"
#include "amc7812conf.h"
#include "amc7812err.h"

enum{
  SPI_BLOCK_SIZE = 3,
};

// GLOBAL VARIABLES

// recall function pointer, best I can do is bitshifts,
// I cant sometimes return a signed int and other times an unsigned
uint16_t (*recallFunction)(uint16_t);

// AMC configuration register 0&1
uint16_t amc_conf[2];
// ADC status register, hold enabled ADCs 
uint16_t adc_status;

// FUNCTION DECLARATIONS
//int16_t amc7812_WriteDACGain( uint16_t value );
//int16_t amc7812_ReadDACGain();
//uint8_t amc7812_EnableADCs();

//=============================================================================
//=============================================================================
//  UTILITY FUNCTIONS
//=============================================================================
//=============================================================================

// amc7812 spi transfer frame
// value in SPDR is returned after sequence
uint16_t amc7812_SPI_transfer ( uint8_t cmd[], uint16_t (*recall)(uint16_t) ){
  uint16_t response = 0;

  // assert CS low
  AMC7812_CS_PORT &= ~(1<<AMC7812_CS_PIN);
  // issue read command for address
  for ( uint8_t i = 0; i < SPI_BLOCK_SIZE; i++ ){
    SPDR = cmd[i];
    while (!(SPSR & (1<<SPIF)));
    // shift response and save response (only the final 2 bytes matter)
    response = (response << 8) + SPDR;
  }
  // release CS high
  AMC7812_CS_PORT |= (1<<AMC7812_CS_PIN);

  // process data according to the last command's callback function
  response = (*recallFunction)(response);
  // store the recall function for the the current command for next time
  recallFunction = recall;
  return response;
}

// dummy recall does no processing
uint16_t dummy_recall( uint16_t d ){
  return d;
}

// recall for write operations, returned value is undefined
uint16_t zero_recall( uint16_t d ){
  return 0;
}

// temp recall right shifts the bottom nibble, and returns a fixed precision
// temperature Temperatuer*8
uint16_t temp_recall( uint16_t d ){
  return (uint16_t)(((int16_t)d)>>4); // arithmetic shift preseves 2's complement
}

//=============================================================================
//=============================================================================
//  PUBLIC FUNCTIONS
//=============================================================================
//=============================================================================

// reset device, check the device ID,
// and perform initialization tasks to be defined later
uint8_t amc7812_Init(){
  // reset device
  // reset pulse width - 20 ns, pg. 6
  // time until normal operation - 250 seconds, pg. 6
  // TODO: add normal operation flag
  AMC7812_RST_DDR  |= (1<<AMC7812_RST_PIN);
  AMC7812_RST_PORT &= ~(1<<AMC7812_RST_PIN);
  AMC7812_RST_PORT |= (1<<AMC7812_RST_PIN);

  recallFunction = &dummy_recall;

  _delay_us(100);  // necessary delay determined experimentally

  // read the device ID register and verify that the expected value matches
  amc7812_Read( AMC7812_DEV_ID );
  // dummy read to get the answer
  uint16_t response = amc7812_Read( 0x00 );
  if( response != 0x1220 ){
    return AMC7812_DEV_ID_ERR;
  }

  // set dac gain to low range by default
  uint16_t gain_sp = 0x0000;
  amc7812_WriteDACGain( gain_sp );
  amc7812_ReadDACGain();
  response = amc7812_Read( 0x00 );
  // check that dac gain has been set low
  if( response != gain_sp ){
    return AMC7812_WRITE_ERR;
  }

  // set dac gain to low range by default
  uint16_t pwr_dwn_reg = 0x7FFE;
  amc7812_Write( AMC7812_POWER_DOWN, pwr_dwn_reg );
  amc7812_Read( AMC7812_POWER_DOWN );
  response = amc7812_Read( 0x00 );
  // check that dac gain has been set low
  if( response != pwr_dwn_reg ){
    return AMC7812_WRITE_ERR;
  }

  // set the ADC ref as internal, and continuous read mode
  uint16_t amc_config = (1<<AMC7812_CMODE)|(1<<AMC7812_ADC_REF);
  amc7812_WriteAMCConfig( 0, amc_config );
  // turn on the adcs
  amc7812_EnableADCs();
  amc7812_TriggerADCs();

  return 0;
}

// amc7812 read from reg at address, 2 byte value from reg stored in resposne
// sequence define starting pg 56 of datasheet
// return value is response from previous frame
// responses are pipelined, so need to send a dummy second command to clock
// out response to this command
uint16_t amc7812_Read ( uint8_t addr ){
  uint8_t cmd[] = { ( AMC7812_READ_MASK | addr ), 0x00, 0x00 };
  return amc7812_SPI_transfer( cmd, dummy_recall );
}

// amc7812 write to reg at address, 2 byte value to reg stored in value
// sequence define starting pg 56 of datasheet
// return value is response from previous frame
uint16_t amc7812_Write( uint8_t addr, uint16_t value  ){
  uint8_t cmd[] = { (AMC7812_WRITE_MASK | addr), 
    (uint8_t)(value>>8), 
    (uint8_t)(value) 
  };
  return amc7812_SPI_transfer( cmd, dummy_recall );
}

//==============================================================================
// TEMPERATURE FUNCTIONS
//==============================================================================

// amc7812 read temperature from sensor, 12-bit value
// conversion 0.125 C/LSB, 0x0000 = 0 C, pg. 61
// returned value is fixed precision signed int, result = (Temp * 8)
uint16_t amc7812_ReadTemp( uint8_t sensor ){
  uint8_t cmd[] = { ( AMC7812_READ_MASK | (AMC7812_TEMP_BASE_ADDR) + sensor ), 0x00, 0x00 };
  return amc7812_SPI_transfer( cmd, temp_recall );
}

//==============================================================================
// ADC FUNCTIONS
//==============================================================================

// amc7812 read adc from sensor, 12-bit value
// returned value is 12-bit unsigned
uint16_t amc7812_ReadADC( uint8_t n ){
  uint8_t cmd[] = { ( AMC7812_READ_MASK | (AMC7812_ADC_BASE_ADDR) + n ), 0x00, 0x00 };
  return amc7812_SPI_transfer( cmd, dummy_recall );
}

// enable all ADCs as single ended
// requires two write commands so return the response from the first one
uint16_t amc7812_EnableADC( uint8_t n ){
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
  setval |= adc_status;
  adc_status = setval; // save new status

  uint8_t cmd[] = { ( AMC7812_WRITE_MASK | AMC7812_ADC_0_CONF + n/13 ), 
    (uint8_t)(setval>>8), 
    (uint8_t)setval
  };
  return amc7812_SPI_transfer( cmd, zero_recall );
}

// enable all ADCs as single ended
// requires two write commands so return the response from the first one
uint16_t amc7812_EnableADCs(){
  // first command
  uint8_t cmd[] = { ( AMC7812_WRITE_MASK | AMC7812_ADC_0_CONF ), 0x6D, 0xFF};
  uint16_t response = amc7812_SPI_transfer( cmd, zero_recall );
  // second command
  cmd[0] = ( AMC7812_WRITE_MASK | AMC7812_ADC_1_CONF );
  cmd[1] = 0x70;
  cmd[2] = 0x00;
  amc7812_SPI_transfer( cmd, zero_recall );
  adc_status = 0xFF;
  return response;
}

// enable all ADCs as single ended
// requires two write commands so return the response from the first one
uint16_t amc7812_DisableADCs(){
  // first command
  uint8_t cmd[] = { ( AMC7812_WRITE_MASK | AMC7812_ADC_0_CONF ), 0x00, 0x00};
  uint16_t response = amc7812_SPI_transfer( cmd, zero_recall );
  // second command
  cmd[0] = ( AMC7812_WRITE_MASK | AMC7812_ADC_1_CONF );
  cmd[1] = 0x00;
  cmd[2] = 0x00;
  amc7812_SPI_transfer( cmd, zero_recall );
  adc_status = 0x00;
  return response;
}

//==============================================================================
// DAC FUNCTIONS
//==============================================================================

// amc7812 read dac setpoint, 12-bit value
// returned value is 12-bit unsigned
uint16_t amc7812_ReadDAC( uint8_t n ){
  uint8_t cmd[] = { ( AMC7812_READ_MASK | (AMC7812_DAC_BASE_ADDR) + n ), 0x00, 0x00 };
  return amc7812_SPI_transfer( cmd, dummy_recall );
}

// amc7812 set dac setpoint, 12-bit value
// returned value is undefined
uint16_t amc7812_WriteDAC( uint8_t n, uint16_t value ){
  uint8_t cmd[] = { ( AMC7812_WRITE_MASK | (AMC7812_DAC_BASE_ADDR) + n ), 
    0x0F & (uint8_t)(value>>8), 
    (uint8_t)value 
  };
  return amc7812_SPI_transfer( cmd, zero_recall );
}

// amc7812 read dac gain, 12-bit value
// returned value is 12-bit
uint16_t amc7812_ReadDACGain(){
  uint8_t cmd[] = { ( AMC7812_READ_MASK | AMC7812_DAC_GAIN ), 0x00, 0x00 };
  return amc7812_SPI_transfer( cmd, dummy_recall );
}

// amc7812 set dac gain, 12-bit value
// value a bitwise mask for the channel gains
// Gain(0) = 2*Vref
// Gain(1) = 5*Vref
uint16_t amc7812_WriteDACGain( uint16_t value ){
  uint8_t cmd[] = { ( AMC7812_WRITE_MASK | AMC7812_DAC_GAIN ), 
    0x0F & (uint8_t)(value>>8), 
    (uint8_t)value 
  };
  return amc7812_SPI_transfer( cmd, zero_recall );
}

//==============================================================================
// CONFIGURATION REGISTERS FUNCTIONS
//==============================================================================

// write to amc config n register
// store the value in a global variable
// cant return response from previous frame and return error
uint8_t amc7812_WriteAMCConfig( uint8_t n, uint16_t config ){
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
  amc7812_Write( (AMC7812_WRITE_MASK|AMC7812_AMC_CONF_0 + n), config);
  return 0;
}

// trigger the adc conversion, if in auto mode CMODE = 1 then the conversions
// happen automatically, if not then a trigger needs to be sent each time the 
// values need to be refreshed
uint16_t amc7812_TriggerADCs(){
  uint16_t config = (amc_conf[0]|(1<<AMC7812_ICONV));
  return amc7812_Write( (AMC7812_WRITE_MASK|AMC7812_AMC_CONF_0), config );
}
