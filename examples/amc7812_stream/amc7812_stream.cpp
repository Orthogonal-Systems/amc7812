#include <Arduino.h>
 
#include "amc7812.h"
#include "amc7812conf.h"

#include <UIPEthernet.h>

#define UDP_TX_PACKET_MAX_SIZE 24           // from normal arduino ethernetudp.h
//#define DHCP 1                              // if static comment
#define LOCAL_PORT  5000

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(169,254,5,12); //Assign my IP adress
//unsigned int localPort = 5000;              //Assign a Port to talk over
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // data buffer
String datReq;                              // String for our data
EthernetUDP Udp; //Define UDP Object
boolean streaming = false;

// define my new class
AMC7812Class AMC7812;

String JSONPair(String member, String value){
  return member + ":" + value + ",";
}

void addReadings(){
  uint8_t spcr = SPCR;                            // save spi settings, before setting up for ADC
  SPCR = AMC7812.GetSPCR();                       // set SPI settings for ADC operations
  uint32_t ts_start = micros();                   // time start at start, us
  uint8_t conv_success = AMC7812.ReadADCs();      // perform conversion cycle on active ADCs
  uint16_t* readings = AMC7812.GetADCReadings();  // retrieve results of the read
  uint16_t adc_gains = AMC7812.GetADCGains();     // get ADC gains
  SPCR = spcr;  // leave no trace

  // build up json formated data packet to send
  Udp.print("{");
  Udp.print( JSONPair("time_us", String(ts_start)) );
  Udp.print( JSONPair("status", String(AMC7812.GetADCStatus())) );
  Udp.print( JSONPair("avref", AMC7812_AVREF) ); // analog voltage reference
  Udp.print( JSONPair("gain", String(adc_gains)) );
  if( conv_success > 0 ){
    Udp.print("readings:{");
    for(uint8_t i=0; i<AMC7812_ADC_CNT; i++){
      uint16_t gain = AMC7812_ADC_GAIN_HIGH;
      if(!(adc_gains & (1<<i))){
        gain = AMC7812_ADC_GAIN_LOW;
      }
      Udp.print( JSONPair(String(i), String(gain * readings[i])) );
    }
    Udp.print("}}");
  } else {
    Udp.print( JSONPair("error", String(conv_success)) );
    Udp.print("}");
  }
}

void setup ()
{
  int success;
  Serial.begin(9600);

  // initialize ADC device
  uint8_t ret = AMC7812.begin();
  while ( ret ){
    Serial.print("Init of AMC7812 failed code: 0x");
    Serial.println(ret, HEX);
    delay(1000);
    ret = AMC7812.begin();
  }
  // enter triggered mode
  AMC7812.SetTriggeredMode();
  Serial.println("AMC7812 device initialized");

  // initialize ethernet device
  Serial.println("Establishing ethernet connection");
  do{

#ifdef DHCP
    if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
      // no point in carrying on, so do nothing forevermore:
      for(;;)
        ;
    }
#else
    Ethernet.begin(mac, ip); //Initialize Ethernet with static ip
#endif

    Serial.print("My IP address: ");
    for (byte thisByte = 0; thisByte < 4; thisByte++) {
      // print the value of each byte of the IP address:
      Serial.print(Ethernet.localIP()[thisByte], DEC);
      Serial.print("."); 
    }

    success = Udp.begin(LOCAL_PORT); //Initialize Udp
    Serial.print("\ninitialize: ");
    Serial.println(success ? "success" : "failed");
    if(!success){
      delay(1500); //delay
    }
  } while (!success);
}

void loop(){
  int packetSize = Udp.parsePacket(); //Read the packetSize
  if (packetSize>0){
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
    String datReq(packetBuffer); //Convert packetBuffer array to string datReq
    //Serial.println("Received packet");
    //Serial.println(datReq);

    // send start command to begin streaming
    if(datReq=="start"){
      Serial.println("beginning stream");
      streaming = true;
    }

    // send stop command to stop streaming
    if(datReq=="stop"){
      Serial.println("halting stream");
      streaming = false;
      Udp.stop();           // close connection
      Udp.begin(LOCAL_PORT); // wait for next connection
    }
    memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
    //Udp.flush();
  }
  if(streaming){
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    addReadings();   //Send string back to client 
    Udp.endPacket(); //Packet has been sent
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
