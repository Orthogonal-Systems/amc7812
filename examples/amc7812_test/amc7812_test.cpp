#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include <Arduino.h>
//#include <Wire.h>
 
#include "amc7812.h"
#include "amc7812conf.h"

#define READ_ADC_REG 0
#define READ_BLOCK_SIZE 512

enum {
 BLINK_DELAY_MS = 500,
};

const char seperator = '+';

#ifdef AMC7812_DAV_INT
volatile uint8_t dav_flag = 0;
// catch DAV dips
ISR(INT0_vect){
  dav_flag = 1;
}

// TODO: make this less dependent on specific hardware
void setup_dav(){
  // setting up dav pin as interrupt
  AMC7812_DAV_DDR &= ~(1<<AMC7812_DAV_PIN);
  EIMSK = 1<<AMC7812_DAV_INT; //enable int0
  EICRA = (1<<ISC00); // trigger int0 on change
  sei();
}
#endif

// define my new class
AMC7812Class AMC7812;

void setup ()
{
  Serial.begin(9600);

  // initialize device
  uint8_t ret = AMC7812.begin();
  while ( ret ){
    Serial.print("Init of AMC7812 failed code: 0x");
    Serial.println(ret, HEX);
    delay(1000);
    ret = AMC7812.begin();
  }
  Serial.println("AMC7812 device initialized.");

#ifdef AMC7812_DAV_INT
  setup_dav();
  Serial.println("waiting for dav trigger");
  // have to hook up dav to get this working
  dav_flag = 0;

  while(!dav_flag); // wait for dav trigger
  dav_flag = 0;
  Serial.println("passed dav trigger, flag reset");

  cli(); // clear global interrupts since it messes with the spi timing
#endif
  // TODO: add dav polling

  // list of registers and default responses for testing
  // last byte is for dummy read
  uint8_t registers[] = { AMC7812_DEV_ID, AMC7812_STATUS, AMC7812_AMC_CONF_0, AMC7812_AMC_CONF_1, AMC7812_ADC_GAIN, AMC7812_DAC_GAIN, AMC7812_TEMP_CONF, 0x0000 };

  // send first read
  uint16_t resp[sizeof(registers)-1];
  for( uint8_t i = 0; i < sizeof(registers)-1; i++ ){
    resp[i]=0;
  }

  Serial.println("Beginning reads");
  AMC7812.Read( registers[0] );
  for( uint8_t i = 1; i < sizeof(registers); i++ ){
    resp[i-1] = AMC7812.Read( registers[i] );
  }

  for( uint8_t i = 0; i < sizeof(registers)-1; i++ ){
    Serial.println(seperator);
    Serial.print("attempting to read register from amc7812: 0x");
    Serial.println(registers[i], HEX);
    Serial.print("response:  0x");
    Serial.println(resp[i], HEX);
    //Serial.println(i, HEX);
    Serial.println(seperator);
  }
}

void loop(){
  delay(5000);

  uint8_t dac_chs[] = { 0, 2 };
  int16_t dac_vals[] = {0x0FFF, 0x07FF};

  Serial.print(seperator);
  Serial.print(seperator);
  Serial.print(seperator);
  for( uint8_t i = 0; i < sizeof(dac_chs); i++ ){
    Serial.printf("attempting to set DAC Ch %.2d to %d mV.\n", dac_chs[i], dac_vals[i] + dac_vals[i]/5 );
  }
  
  // send first read
  for( uint8_t i = 0; i < sizeof(dac_chs); i++ ){
    AMC7812.WriteDAC( dac_chs[i], dac_vals[i] );
  }

  uint16_t adc_vals[AMC7812_ADC_CNT];
  for(uint8_t j=0; j < 2; j++){
    AMC7812.ReadADC( 0 );
    // just do a dummy read of the next register to round it out
    for( uint8_t i = 1; i <= AMC7812_ADC_CNT; i++ ){
      adc_vals[i-1] = AMC7812.ReadADC( i );
    }
    Serial.print(seperator);
    for( uint8_t i = 0; i < AMC7812_ADC_CNT; i++ ){
      Serial.printf("ADC[%.2d]: 0x%.3X\n", i, adc_vals[i]);
    }
    Serial.println(seperator);
    _delay_ms(5000);
  }

  Serial.printf("\nstopping all channel reads\n");
  AMC7812.DisableADCs();
  Serial.printf("switching to channel %d ADC reads only\n",READ_ADC_REG);
  AMC7812.EnableADC(READ_ADC_REG);

  uint16_t adc_vals2[READ_BLOCK_SIZE];
  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    adc_vals[i] = 0;
  }

  // retrigger since I turned them off
  AMC7812.TriggerADCsInternal();
  delay(1);

  AMC7812.ReadADC(READ_ADC_REG);
  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    adc_vals2[i] = AMC7812.ReadADC(READ_ADC_REG);
    // Ac update rate is faster than read rate
    //dav_flag = 0;
    //while(!dav_flag); // wait until there is a new value
  }

  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    Serial.println(adc_vals2[i]);
  }
<<<<<<< HEAD
  
  AMC7812.EnableADCs();
  AMC7812.TriggerADCsExternal();
=======

  AMC7812.EnableADCs();
>>>>>>> cf0001054aebbac2ce2db346108d5c737bab9866
}

// normal arduino main function
int main(void){
  init();

  setup();

  for(;;){
    loop();
    if (serialEventRun) serialEventRun();
  }
  return 0;
}
