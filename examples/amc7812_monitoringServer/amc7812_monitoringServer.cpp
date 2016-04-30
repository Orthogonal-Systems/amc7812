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

#include "amc7812.h"
#include "amc7812conf.h"
#include "amc7812err.h"

#include "zmqduino.h"   // zmq interface
#include "monitoringServer.h" // monitoringServer interface

#define DHCP 0

//uint8_t channels = AMC7812_ADC_CNT; //16
uint8_t channels = AMC7812_ADC_CNT-2; //14 (ran out of space in 256 bit buffer, need to compress data or switch back to integers)
//uint8_t dataEntrySize = 5; // 16 bits ~> 65,000 -> 5 digits
//uint8_t dataEntrySize = 4; // 12 bits ~> 4,000 -> 4 digits
uint8_t dataEntrySize = 7; // for fp values

// TODO: make macro for size of buffer
char zmq_buffer[256]={0}; //!< buffer for zmq communication, needs to fit dataPacket

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF};

// initialize the library instance:
EthernetClient client;
#if !DHCP
//IPAddress ip    (169,254,5,10);
IPAddress ip    (169,254,5,12);
#endif
ZMQSocket ZMQPush(client, zmq_buffer, PUSH);
uint8_t useFractionalSecs = 1;
DataPacket packet( channels
    , (char *)"ADC2" // stream name
    , 4 // length of the stream name
    , dataEntrySize // max length of decimal character string to use
    , zmq_buffer + ZMQ_MSG_OFFSET // communication buffer after ZMQ header
    , useFractionalSecs // send sub-second timestamp info
);

// address for registration and sending of data
IPAddress data_server(169,254,5,183);
int reg_port = 5556;
int mes_port = 5557;

EthernetUDP Udp;
// address for ntp server sync
IPAddress ntp_server(169,254,5,183);
int ntp_port = 8888;  // local port to listen for UDP packets

// mega pin 13
uint8_t lastTrig = 1;
uint8_t trigpin = AMC7812_DIO0_ARDUINO;

// define my new class
AMC7812Class AMC7812;
// linear calibrations for channels
float m[AMC7812_ADC_CNT] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0 // NC
  ,1.20886,1.26525  // X
  ,1.09459,1.20886  // Y
  ,0.156906,1.00662 // Z
  ,1.0,1.0}; // NC
float b[AMC7812_ADC_CNT] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 // NC
  ,0.0136734,0.0107174  // X
  ,0.0035753,0.0136734  // Y
  ,0.0031166,0.0934562  // Z
  ,0.0,0.0};  // NC

uint8_t longerSyncPeriod = 0; // state tracker

void sendNTPpacket(IPAddress &address);
time_t getNtpTime();

uint8_t addReadings(){
  uint8_t spcr = SPCR;                            // save spi settings, before setting up for ADC
  uint8_t spsr = SPSR;                            // save spi settings, before setting up for ADC
  SPCR = AMC7812.GetSPCR();                       // set SPI settings for ADC operations
  SPSR = AMC7812.GetSPSR();                       // set SPI settings for ADC operations
  time_t ts = now(0);                             // timestamp at start (dont allow ntp sync), ms
  uint8_t conv_success = AMC7812.ReadADCs();      // perform conversion cycle on active ADCs
  uint16_t* readings = AMC7812.GetADCReadings();  // retrieve results of the read
  SPCR = spcr;  // leave no trace
  SPSR = spsr;  // leave no trace

  // separate 64b timestamp to 32b second and fractional second components
  uint32_t ts_sec = toSecs(ts);
  uint32_t ts_fsec = toFracSecs(ts);
  
  float voltages[channels];
  uint8_t allZeros = 0;
  for( uint8_t i=0; i<channels; i++ ){
    if( readings[i] == 0 ){
      allZeros++;
    }
    voltages[i] = conv_success ? 0 : (5.0*(float)readings[i]/(4096.0))*m[i] + b[i];
  }
  
  digitalWrite(AMC7812_COMM_LED_ARDUINO, HIGH);
  uint8_t len = packet.preparePacket( ts_sec, ts_fsec, voltages );
  Serial.write((uint8_t*)(zmq_buffer+ZMQ_MSG_OFFSET),len);
  Serial.println();
  ZMQPush.sendZMQMsg(len);
  if (allZeros){
    return AMC7812_TIMEOUT_ERR;
  }
  digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);
  return conv_success;
}

uint8_t setup_DAQ(){
  uint8_t spcr = SPCR;                            // save spi settings, before setting up for ADC
  uint8_t spsr = SPSR;                            // save spi settings, before setting up for ADC
  SPCR = AMC7812.GetSPCR();                       // set SPI settings for ADC operations
  SPSR = AMC7812.GetSPSR();                       // set SPI settings for ADC operations

  digitalWrite(AMC7812_IDLE_LED_ARDUINO, LOW);
  digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);

  Serial.print(F("\ninitializing AMC7812..."));
  uint8_t ret = AMC7812.begin();
  while ( ret ){
    Serial.print(F("Init of AMC7812 failed code: 0x"));
    Serial.println(ret, HEX);
    delay(1000);
    ret = AMC7812.begin();
  }
  // enter triggered mode
  AMC7812.SetTriggeredMode();
  //AMC7812.DisableADCs();
  //for( uint8_t i=8; i<=13; i++){
  //  AMC7812.EnableADC(i);
  //}
  
  SPCR = spcr;  // leave no trace
  SPSR = spsr;  // leave no trace

  Serial.println(F("AMC7812 device initialized"));
  return ret;
}

void setup_ethernet(){
  // set up ethernet chip
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
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
  Serial.println(F("Setting up REQ socket"));
  ZMQSocket ZMQReq( client, zmq_buffer, REQ );
  int16_t len;
  do{
    uint8_t err = 0;
    len = -1;
    if( ZMQReq.connect( data_server, reg_port ) ){
      client.stop(); // TODO: deal with this better
      Serial.println("Cant connect to server");
      err = 1;
    }
    if(!err){
      // register datastream with server
      Serial.println(F("Registering data stream..."));
      len = packet.registerFloatStream();
      ZMQReq.sendZMQMsg(len);
      len = ZMQReq.recv();
      if( len < 0 ){
        Serial.println(F("negative len returned"));
        client.stop();
      }
    }
  } while( len < 0 );
  // check that we got the expected response from the moitoring server
  if( len != 7 ){
    Serial.println(F("Invalid Response from server, length: "));
    Serial.println(len);
    for(;;)
      ;
  }
  // disconnect from registering port
  Serial.println(F("Disconnecting REQ socket..."));
  client.stop();
}

void setup_data_stream(){
  // setup push socket
  Serial.println(F("Setting up PUSH socket..."));
  if( ZMQPush.connect( data_server, mes_port ) ){
    Serial.println(F("oops"));
    client.stop(); // TODO: deal with this better
    for(;;)
      ;
  }
  Serial.println(F("Starting"));
  Serial.println(F("Data"));
  Serial.println(F("Stream"));
}


void setup() {
  // minimal SPI bus config (cant have two devices being addressed at once)
  //PORTG |= (1<<0);  // set AMC7812 CS pin high if connected
  digitalWrite(AMC7812_CS_ARDUINO_PIN, HIGH); // set AMC CS pin high 
  digitalWrite(SS, HIGH); // set ENCJ CS pin high 
  pinMode(trigpin, INPUT);

  Serial.begin(115200); //Turn on Serial Port for debugging

  uint8_t ret = setup_DAQ();
  digitalWrite(AMC7812_COMM_LED_ARDUINO, HIGH);
  setup_ethernet();
  setup_ntp();
  register_stream();
  digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);

  delay(1000); // increase stability
  digitalWrite(AMC7812_COMM_LED_ARDUINO, HIGH);
  setup_data_stream();
  digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);
  delay(1000);
}

void loop() {
  digitalWrite(AMC7812_IDLE_LED_ARDUINO, HIGH);
  digitalWrite(AMC7812_COMM_LED_ARDUINO, HIGH);
  Ethernet.maintain();

  now();  // see if its time to sync
//  if( !client.connected() ){
//    Serial.println(F("Client disconnected, attempting reconnect..."));
//    client.stop();
//    setup_data_stream();
//  }
  digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);

  // if we havent extended the time period yet,
  // and if there is a non-zero drift correction
  // then make the period longer
  if( (!longerSyncPeriod) & getDriftCorrection() ){
    setSyncInterval(72000);  // every 2 hours
    longerSyncPeriod=1;
  }

  uint8_t newTrig = digitalRead(trigpin);
  // trig on low to high trigger
  //if( lastTrig && !newTrig ){
  // trig on high to low trigger
  if( !lastTrig && newTrig ){
    digitalWrite(AMC7812_IDLE_LED_ARDUINO, LOW);
    //Send string back to client 
    if(addReadings() == AMC7812_TIMEOUT_ERR){
      setup_DAQ();
    }
  }
  lastTrig = newTrig;

  // check for incoming packet, do stuff if we need to
  uint8_t len = client.available();
  if( len ){
    digitalWrite(AMC7812_COMM_LED_ARDUINO, HIGH);
    len = ZMQPush.read(); // process header and get get actual mesg length
    digitalWrite(AMC7812_COMM_LED_ARDUINO, LOW);
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
  
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

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
