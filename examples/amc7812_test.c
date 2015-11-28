#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
 
#include "amc7812.h"
#include "amc7812conf.h"

#include "usart.h"

#define READ_ADC_REG 0
#define READ_BLOCK_SIZE 512

enum {
 BLINK_DELAY_MS = 500,
};

// initialize the SPI pins
void spi_init_master(){
  // set MOSI & SCK & CS as output
  AMC7812_SPI_DDR = _BV(AMC7812_SPI_SCLK) | _BV(AMC7812_SPI_MOSI);
  AMC7812_CS_DDR |= _BV(AMC7812_CS_PIN);
  // setup spi status register, SPI mode 1
  // prescalar 128, f/128
  //SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0)|(1<<CPHA);
  // prescalar 2, f/2
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA); // 4x prescalar
  SPSR = (1<<SPI2X); // double time
}
/*
volatile uint8_t dav_flag = 0;
// catch DAV dips
ISR(INT0_vect){
  dav_flag = 1;
}
*/

int main (void)
{
  const char seperator[] = "++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  USART0Init();

  // initialize SPI as master
  spi_init_master();

  /*
  // setting up dav pin as interrupt
  DDRD &= ~(1<<2);
  EIMSK = 1<<INT0; //enable int0
  EICRA = (1<<ISC00); // trigger int0 on change
  sei();

  // setup PWM on pin 6
  TCCR0A = (0<<WGM01)|(1<<WGM00);  // PWM, phase correct
  //TCCR0B = (1<<CS00);              //no prescaler
  //TCCR0B = (1<<CS01)|(1<<CS00);      //64 prescaler
  TCCR0B = (1<<CS02);               //256 prescaler
  OCR0A = 0xC0; //set 75% duty cycle
  DDRD |= (1<<DDD6);
*/
  printf("\n\nhello\n");

  // initialize device
  uint8_t ret = amc7812_Init();
  if( ret ){
    printf("Init of AMC7812 failed with error code: 0x%.2X", ret);
    return 0;
  }
  printf("AMC7812 device initialized.\n");
  /*
  dav_flag = 0;

  while(!dav_flag); // wait for dav trigger
  dav_flag = 0;
  */
  printf("passed first dav trigger, flag reset");
  printf("aamc_conf_0 sent: %.4X\n", (1<<AMC7812_CMODE)|(1<<AMC7812_ADC_REF));
  // list of registers and default responses for testing
  // last byte is for dummy read
  uint8_t registers[] = { AMC7812_DEV_ID, AMC7812_STATUS, AMC7812_AMC_CONF_0, AMC7812_AMC_CONF_1, AMC7812_ADC_GAIN, AMC7812_DAC_GAIN, AMC7812_TEMP_CONF, 0x0000 };
  uint16_t defaultResp[] = {0x1220, 0x0000, 0x2000, 0x0070, 0xffff, 0x0000, 0x003c };

  printf("%d commands entered\n",sizeof(registers));

  uint16_t resp[sizeof(registers)-1];
  for( uint8_t i = 0; i < sizeof(registers)-1; i++ ){
    resp[i]=0;
  }

  // send first read
  amc7812_Read( registers[0] );
  for( uint8_t i = 1; i < sizeof(registers); i++ ){
    resp[i-1] = amc7812_Read( registers[i] );
  }

  for( uint8_t i = 0; i < sizeof(registers)-1; i++ ){
    printf(seperator);
    printf("attempting to read 0x%.2X register from amc7812\n", registers[i]);
    printf("default response: [0x%.4X]\n", defaultResp[i]);
    printf("response        :  0x%.4X\n", resp[i]);
    printf(seperator);
  }

  //while(!dav_flag); // wait for dav trigger, if this is continuous mode then we should see new data
  

  uint8_t dac_chs[] = { 0, 2 };
  int16_t dac_vals[] = {0x0FFF, 0x07FF};

  printf(seperator);
  printf(seperator);
  printf(seperator);
  for( uint8_t i = 0; i < sizeof(dac_chs); i++ ){
    printf("attempting to set DAC Ch %d to %d mV\n", dac_chs[i], dac_vals[i]*5000/0x1000 );
  }
  
  // send first read
  for( uint8_t i = 0; i < sizeof(dac_chs); i++ ){
    amc7812_WriteDAC( dac_chs[i], dac_vals[i] );
  }

  uint16_t adc_vals[AMC7812_ADC_CNT];
  for(uint8_t i=0; i < 4; i++){
    amc7812_ReadADC( 0 );
    // just do a dummy read of the next register to round it out
    for( uint8_t i = 1; i <= AMC7812_ADC_CNT; i++ ){
      adc_vals[i-1] = amc7812_ReadADC( i );
    }
    printf(seperator);
    for( uint8_t i = 0; i < AMC7812_ADC_CNT; i++ ){
      printf("ADC[%.2d]: 0x%.3X\n", i, adc_vals[i]);
    }
    printf(seperator);
    _delay_ms(5000);
  }

  printf("stopping all channel reads\n");
  amc7812_DisableADCs();
  printf("switching to channel %d ADC reads only\n",READ_ADC_REG);
  amc7812_EnableADC(READ_ADC_REG);

  uint16_t adc_vals2[READ_BLOCK_SIZE];
  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    adc_vals[i] = 0;
  }

  amc7812_TriggerADCs();

  amc7812_ReadADC(READ_ADC_REG);
  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    adc_vals2[i] = amc7812_ReadADC(READ_ADC_REG);
  }

  for( uint16_t i = 0; i < READ_BLOCK_SIZE; i++ ){
    printf("%d,\n",adc_vals2[i]);
  }

  return 0;
}
