/*

This is a program that reads data when a trigger is recieved from an AMC7812 ADC
chip and uploads it to the monitoring server

created 18 Feb 2015 
by Matt Ebert

You need an Ethernet Shield (ENCJ) and an AMC7812 shield

Ethernet is handled by the uIPEthernet library by (Norbert Truchsess) which uses 
the uIP tcp stack by Adam Dunkels.

REQ and PUSH ZeroMQ sockets are emulated

 */

#include <Arduino.h>
#include <UIPEthernet.h>
#include <UIPClient.h>
#include <TimeLib.h>
#include <Wire.h> // I2C
#include <avr/wdt.h>
#include <util/delay.h>

#include "amc7812.h"
#include "amc7812conf.h"
#include "amc7812err.h"
#include "tpic2810.h"  // input offset driver

#include "origin.h"   // data server interface
#include "datapacket.h"

//#include "frontpanel.h" // frontpanel led drivers

#define DHCP 1
#define EXT_TRIG 1
//#define DEBUG

// DATA COLLECTION CONFIG /////////////////////////////////////////////////////
AMC7812Class AMC7812;
uint8_t channels = AMC7812_ADC_CNT;
uint8_t lastTrig = 1;
uint32_t nextTrig = 0;
uint8_t trigpin = 12;  // AMC7812_DIO0_ARDUINO;
const unsigned long postingInterval = 100;  //delay between updates (in milliseconds)
const unsigned long secondTrigInterval = 10;  //delay between second trig pass (in milliseconds)
///////////////////////////////////////////////////////////////////////////////

// GENERIC ETHERNET INTERFACE CONFIG //////////////////////////////////////////
EthernetClient client;
IPAddress ip    (192,168,0,101);
char buffer[256]={0};
///////////////////////////////////////////////////////////////////////////////

// ORIGIN DATA SERVER INTERFACE CONFIG ////////////////////////////////////////
// fill in an available IP address on your network here,
// for manual configuration:
IPAddress data_server(128,104,160,152);
int reg_port = 5558;
int mes_port = 5559;

// origin data server interface
Origin origin( client
    , data_server
    , reg_port
    , mes_port
    , (char *)"FNODE_ADCS"   // stream name
    , 10        // stream name length (max 10)
    , 1         // use fractional seconds
    , channels  // maximum channels
    , channels  // channels used
    , (char *)DTYPE_UINT16
    , INT16_TYPE_SIZE
    , buffer
);
///////////////////////////////////////////////////////////////////////////////

// NTP SERVER CONFIG //////////////////////////////////////////////////////////
EthernetUDP Udp;
// address for ntp server sync
IPAddress ntp_server(128,104,160,150);
int ntp_port = 8888;  // local port to listen for UDP packets
void sendNTPpacket(IPAddress &address);
time_t getNtpTime();
uint8_t longerSyncPeriod = 0; // state tracker
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
///////////////////////////////////////////////////////////////////////////////

uint8_t addReadings(int16_t (&reads)[AMC7812_ADC_CNT]){
  uint8_t conv_success;
  uint8_t spcr = SPCR;                         // save spi settings, before setting up for ADC
  uint8_t spsr = SPSR;                         // save spi settings, before setting up for ADC
  SPCR = AMC7812.GetSPCR();                    // set SPI settings for ADC operations
  SPSR = AMC7812.GetSPSR();                    // set SPI settings for ADC operations
  conv_success = AMC7812.ReadADCs();         // perform conversion cycle on active ADCs
  uint16_t* temp = AMC7812.GetADCReadings(); // retrieve results of the read
  for( uint8_t j=0; j<channels; j++ ){
    reads[j] = temp[j];
  }
  SPCR = spcr;  // leave no trace
  SPSR = spsr;  // leave no trace

  uint8_t allZeros = 1;
  for( uint8_t i=0; i<channels; i++ ){
    if( reads[i] != 0 ){
      allZeros=0;
      break;
    }
  }
  
  if (allZeros){
    return AMC7812_TIMEOUT_ERR;
  }
  return conv_success;
}

uint8_t setup_offsets(){
  setup_tpic2810();
  // offsets 0 -> 2.5 V offset added
  // offsets 1 -> 0.0 V offset
  uint8_t ret = set_tpic2810_all(IOFFSET_0_ADDR, 0xC7);
  ret |= set_tpic2810_all(IOFFSET_1_ADDR, IOFFSET_OFF);
  return ret;  // 0 if ok
}


uint8_t setup_DAQ(){
  uint8_t spcr = SPCR;       // save SPI settings, before setting up for ADC
  uint8_t spsr = SPSR;       // save SPI settings, before setting up for ADC
  SPCR = AMC7812.GetSPCR();  // set SPI settings for ADC operations
  SPSR = AMC7812.GetSPSR();  // set SPI settings for ADC operations

  Serial.print(F("\ninitializing AMC7812..."));
  uint8_t ret = AMC7812.begin();
  while ( ret ){
    Serial.print(F("Init of AMC7812 failed code: 0x"));
    Serial.println(ret, HEX);
    for( uint8_t i=0; i<10; i++){
      delay(100);
    }
    ret = AMC7812.begin();
  }
  // enter triggered mode
  AMC7812.SetTriggeredADCMode();
  
  SPCR = spcr;  // leave no trace
  SPSR = spsr;  // leave no trace

  Serial.println(F("AMC7812 device initialized"));
  return ret;
}

void setup_ethernet(){
  // set up ethernet chip
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01};
#if DHCP && UIP_UDP // cant use DHCP without using UDP
  Serial.println(F("DHCP..."));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed DHCP"));
    for(;;)
      ;
  }
#else
  Serial.println(F("STATIC..."));
  Ethernet.begin(mac,ip);
#endif

  Serial.println(Ethernet.localIP());
  Serial.println(Ethernet.subnetMask());
  Serial.println(Ethernet.gatewayIP());
  Serial.println(Ethernet.dnsServerIP());
}

void setup_ntp(){
  // setup NTP sync service
  Udp.begin(ntp_port);
  Serial.println(F("Waiting for NTP sync"));
  setSyncInterval(600); // initial one 10 minutes to take care of most of the drift
  setSyncProvider(getNtpTime);
}

void register_stream(){
  // setup request socket
  Serial.println(F("Registering stream with server."));
  uint8_t err;
  do{
    err = origin.registerStream();
    if(err==ERR_SERVER_RESP){
      Serial.println(F("Server error."));
    }
    if(err==ERR_SERVER_LEN){
      Serial.println(F("Invalid Response length from server."));
    }
  } while( err );
  Serial.println(F("Disconnecting REQ socket..."));
}

void setup_data_stream(){
  Serial.println(F("Setting up data stream..."));
  origin.setupDataStream();
  Serial.println(F("Starting"));
  Serial.println(F("Data"));
  Serial.println(F("Stream"));
}

void setup() {
  // minimal SPI bus config (cant have two devices being addressed at once)
  digitalWrite(AMC7812_CS_ARDUINO_PIN, HIGH); // set AMC CS pin high 
  digitalWrite(SS, HIGH); // set ENCJ CS pin high 
  pinMode(AMC7812_CS_ARDUINO_PIN, OUTPUT);
  pinMode(SS, OUTPUT);
  //pinMode(trigpin, INPUT);

  Serial.begin(115200); //Turn on Serial Port for debugging

  setup_DAQ();
  setup_offsets();
  setup_ethernet();
  setup_ntp();
  register_stream();

  delay(1000); // increase stability
  setup_data_stream();
  delay(1000);
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  Ethernet.maintain();

  now();  // see if its time to sync

  // if we havent extended the time period yet,
  // and if there is a non-zero drift correction
  // then make the period longer
  if( (!longerSyncPeriod) & getDriftCorrection() ){
    Serial.println(F("syncing to network time."));
    setSyncInterval(72000);  // every 2 hours
    longerSyncPeriod=1;
  }

  uint8_t newTrig = 0;
#if EXT_TRIG
  // TODO: replace with frontpanel function
  newTrig = digitalRead(trigpin); 
#else
  if( millis() > nextTrig ){
    newTrig = 1;
    nextTrig = millis() + secondTrigInterval;
  }
#endif

  if( !lastTrig && newTrig ){  // trig on low to high trigger
    time_t ts = now(0);  // timestamp at start (dont allow ntp sync), ms
    //Serial.println("trigger detected");
    //Send string back to client 
    int16_t readings[AMC7812_ADC_CNT]; // array for low to high trig
    if(addReadings(readings) == AMC7812_TIMEOUT_ERR){
      setup_DAQ();
    }

    int16_t readings2[AMC7812_ADC_CNT]; // array for high to low trig
    // wait for second trigger transition
    while(newTrig){
#if EXT_TRIG
      // TODO: replace with frontpanel function
      newTrig = digitalRead(trigpin); 
#else
      if( millis() > nextTrig ){
        newTrig = 0;
        nextTrig = millis() + postingInterval;
      }
#endif
    }
    if(addReadings(readings2) == AMC7812_TIMEOUT_ERR){
      setup_DAQ();
    }

    // take the first 8 channels from the first trigger readings
    // and the second 8 channels from the second trigger readings
//    for(uint8_t os=AMC7812_ADC_CNT/2; os<AMC7812_ADC_CNT; os++){
//      readings[os] = readings2[os];
//    }


    int16_t readings3[AMC7812_ADC_CNT]; // array for second low to high trig
    while(!newTrig){
#if EXT_TRIG
      // TODO: replace with frontpanel function
      newTrig = digitalRead(trigpin); 
#else
      if( millis() > nextTrig ){
        newTrig = 0;
        nextTrig = millis() + postingInterval;
      }
#endif
    }
    if(addReadings(readings3) == AMC7812_TIMEOUT_ERR){
      setup_DAQ();
    }
    // substract background signal from relevant channels
    // the readings vectors are all messed up and I cant figure out why
    // readings is supposed to hold the first trigger
    // readings2 is supposed to hold the second trigger
    // readings3 is supposed to hold the third, but they are all jumbled
    // I had to determine what is ending up where experimentally
    // So beware
    readings2[1] = readings2[1] - readings[1];
    readings2[2] = readings2[2] - readings[2];
    //readings2[8] = readings2[8] - readings3[8];
    readings2[8] = readings2[8] - readings[1];
    // might as well put the backgrounds on some unused channels
    readings2[9] = readings[1];
    readings2[10] = readings[2];


    // separate 64b timestamp to 32b second and fractional second components
    uint32_t ts_sec = toSecs(ts);
    uint32_t ts_fsec = toFracSecs(ts);
    // I don't know why the readings vector doesn't seem to work correctly
    origin.sendPacket( ts_sec, ts_fsec, (int16_t*)readings2 );
  }
  lastTrig = newTrig;

  // check for incoming packet, do stuff if we need to
  uint8_t len = client.available();
  if( len ){
    Serial.println("incoming packet");
    client.read((uint8_t*)buffer, len);
    Serial.write(buffer, len);
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

/*-------- NTP code ----------*/
  
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(ntp_server);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1000) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      unsigned long fracSecs;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      fracSecs =  (unsigned long)packetBuffer[44] << 24;
      fracSecs |= (unsigned long)packetBuffer[45] << 16;
      fracSecs |= (unsigned long)packetBuffer[46] << 8;
      fracSecs |= (unsigned long)packetBuffer[47];
      return ((time_t)(secsSince1900 - 2208988800UL)<<32) + fracSecs;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
