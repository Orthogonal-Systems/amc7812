#include <Arduino.h>
 
#include "frontpanel.h"

enum {
 BLINK_DELAY_MS = 500,
};

void setup (){
  Serial.begin(9600);
  Serial.println("hello");
  frontpanel_setup();
  frontpanel_set_led( IDLE_LED, 1 );
}

void loop(){
  frontpanel_set_led( USER1_LED, 1 );
  delay(1000);
  frontpanel_set_led( USER1_LED, 0 );
  delay(1000);
}
