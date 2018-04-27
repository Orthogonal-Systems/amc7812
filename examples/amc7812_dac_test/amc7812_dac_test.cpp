#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include <Arduino.h>
 
#include "amc7812.h"
#include "amc7812conf.h"

#define READ_ADC_REG 0
#define READ_BLOCK_SIZE 512

enum {
 BLINK_DELAY_MS = 500,
};

const char seperator = '+';

// define my new class
AMC7812Class AMC7812;

void setup ()
{
  Serial.begin(115200);

  // initialize device
  uint8_t ret = AMC7812.begin();
  while ( ret ){
    Serial.print("Init of AMC7812 failed code: 0x");
    Serial.println(ret, HEX);
    delay(1000);
    ret = AMC7812.begin();
  }
  Serial.println("AMC7812 device initialized.");
}

void loop(){
  for( uint16_t i = 0; i < (1 << 12); i++ ){
    AMC7812.WriteDAC(0, i);
  }
  for( uint16_t i = 1; i < (1 << 12); i++ ){
    AMC7812.WriteDAC(0, (1 << 12) - i);
  }
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
