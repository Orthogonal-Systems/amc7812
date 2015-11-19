/*****************************************************************************
*
* Title  : Texas Instruments AMC7812 Integrated ADC & DAC Driver
* Author : Matthew Ebert (c)2015
* Copyright: GPL V3
*
*This driver provides initialization and transmit/receive
*functions for the TI AMC7812 integrated ADC and DAC.
*
*****************************************************************************/

#ifndef AMC7812_H
#define AMC7812_H

// AMC7812 Register Mappings
// from page 60 of the datasheet SBAS513E
// defualt values are for SPI are shown in []

// TEMPERATURE
#define AMC7812_TEMP_BASE_ADDR   0x00 // base address for temperature sensors
#define AMC7812_TEMP_MAX_ADDR    0x02 // max address for termparture sensors
#define AMC7812_TEMP_SENS_CNT    3    // number of teperature sensors

#define AMC7812_TEMP_LOCAL       0x00 // (R) address for local temperature sensor
#define AMC7812_TEMP_D1          0x01 // (R) address for D1 temperature sensor
#define AMC7812_TEMP_D2          0x02 // (R) address for D2 temperature sensor
#define AMC7812_TEMP_CONF        0x0a // (R/W) temperature configuration [0x003C]
#define AMC7812_TEMP_CONV        0x0b // (R/W) temperature conversion rate [0x0007]
#define AMC7812_TEMP_D1_ETA      0x21 // (R/W) eta-factor correction for D1
#define AMC7812_TEMP_D2_ETA      0x22 // (R/W) eta-factor correction for D2
// ADCs
#define AMC7812_ADC_BASE_ADDR    0x23 // (R) ADC 0 address
#define AMC7812_ADC_MAX_ADDR     0x32 // (R) ADC n address
#define AMC7812_ADC_CNT          16   // number of onboard ADCs
// DACs
#define AMC7812_DAC_BASE_ADDR    0x33 // (R/W) DAC 0 Data
#define AMC7812_DAC_MAX_ADDR     0x3e // (R/W) DAC n Data
#define AMC7812_DAC_CNT          12   // number of onboard DACs
// DAC-CLRs
#define AMC7812_DAC_CLR_BASE_ADDR 0x3f // (R/W) DAC n CLR Setting [0x0000]
#define AMC7812_DAC_CLR_MAX_ADDR  0x4a // (R/W) DAC 11 CLR Setting [0x0000]
// OTHER STUFF
#define AMC7812_GPIO        0x4b // (R/W) GPIO [0x00FF]
#define AMC7812_AMC_CONF_0  0x4c // (R/W) AMC configuration 0 [0x2000]
#define AMC7812_AMC_CONF_1  0x4d // (R/W) AMC configuration 1 [0x0070]
#define AMC7812_AMC_CONF_CNT   2 // number of conversion registers
#define AMC7812_ALARM_CTRL  0x4e // (R/W) Alarm control [0x0000]
#define AMC7812_STATUS      0x4f // (R) Status
#define AMC7812_ADC_0_CONF  0x50 // (R/W) ADC 0 configuration [0x0000]
#define AMC7812_ADC_1_CONF  0x51 // (R/W) ADC 1 configuration [0x0000]
#define AMC7812_ADC_GAIN    0x52 // (R/W) ADC Gain [0xffff]
#define AMC7812_AUTO_DAC_CLR_SRC  0x53 // (R/W) auto dac clear source [0x0004]
#define AMC7812_AUTO_DAC_CLR_EN   0x54 // (R/W) auto dac clear enable [0x0000]
#define AMC7812_SW_DAC_CLR  0x55 // (R/W) software dac clear [0x0000]
#define AMC7812_HW_DAC_CLR_EN_0   0x56 // (R/W) hardware dac clear enable 0 [0x0000]
#define AMC7812_HW_DAC_CLR_EN_1   0x57 // (R/W) hardware dac clear enable 1 [0x0000]
#define AMC7812_DAC_CONF    0x58 // (R/W) DAC configuration
#define AMC7812_DAC_GAIN    0x59 // (R/W) DAC gain
#define AMC7812_IN_0_HT     0x5a // (R/W) input 0 high threshold [0xffff]
#define AMC7812_IN_0_LT     0x5b // (R/W) input 0 low threshold [0x0000]
#define AMC7812_IN_1_HT     0x5c // (R/W) input 1 high threshold [0xffff]
#define AMC7812_IN_1_LT     0x5d // (R/W) input 1 low threshold [0x0000]
#define AMC7812_IN_2_HT     0x5e // (R/W) input 2 high threshold [0xffff]
#define AMC7812_IN_2_LT     0x5f // (R/W) input 2 low threshold [0x0000]
#define AMC7812_IN_3_HT     0x60 // (R/W) input 3 high threshold [0xffff]
#define AMC7812_IN_3_LT     0x61 // (R/W) input 3 low threshold [0x0000]
#define AMC7812_TEMP_CHIP_HT  0x62 // (R/W) chip temperature high threshold [0x07ff]
#define AMC7812_TEMP_CHIP_LT  0x63 // (R/W) chip temperature low threshold [0x0800]
#define AMC7812_TEMP_D1_HT  0x64 // (R/W) sensor 1 temperature high threshold [0x07ff]
#define AMC7812_TEMP_D1_LT  0x65 // (R/W) sensor 1 temperature low threshold [0x0800]
#define AMC7812_TEMP_D2_HT  0x66 // (R/W) sensor 2 temperature high threshold [0x07ff]
#define AMC7812_TEMP_D2_LT  0x67 // (R/W) sensor 2 temperature low threshold [0x0800]
#define AMC7812_HYST_0      0x68 // (R/W) hysteresis 0 [0x0810]
#define AMC7812_HYST_1      0x69 // (R/W) hysteresis 1 [0x0810]
#define AMC7812_HYST_2      0x6a // (R/W) hysteresis 2 [0x2108]
#define AMC7812_POWER_DOWN  0x6b // (R/W) power down [0x0000]
#define AMC7812_DEV_ID      0x6c // (R) device id [0x1220]
#define AMC7812_SW_RST      0x7c // (R/W) software reset

// CONFIGURATION REGISTER BIT MAPPINGS
//
//AMC_CONF_0 settings (1<<setting), see pg 66
#define AMC7812_CMODE       13   // Conversion mode - 0: direct, 1: auto
#define AMC7812_ICONV       12   // Internal conversion bit, starts ADC conversion
#define AMC7812_ILDAC       11   // Load DAC, 1: sync update DAC channels set for SLDAC
#define AMC7812_ADC_REF     10   // ADC Reference, 1: internal, 0: external
#define AMC7812_EN_ALR       9   // Alarm 0: disabled, 1: enabled
#define AMC7812_DAVF         7   // (Read-Only) Data available flag (direct CMODE)
#define AMC7812_GALR         6   // (Read-Only) Global alarm bit
//AMC_CONF_1 settings (1<<setting), see pg 67
#define AMC7812_CONV_RATE_1  9   // ADC conversion rate bit
#define AMC7812_CONV_RATE_0  8   // ADC conversion rate bit
#define AMC7812_CH_FALR_2    7   // False alarm protection bit CH0-3
#define AMC7812_CH_FALR_1    6   // False alarm protection bit CH0-3
#define AMC7812_CH_FALR_0    5   // False alarm protection bit CH0-3
#define AMC7812_TEMP_FALR_1  4   // False alarm protection bit temp monitor
#define AMC7812_TEMP_FALR_0  3   // False alarm protection bit temp monitor
//ALARM_CTRL
#define AMC7812_EALR_0      14   // CH 0 alarm bit, enable to trigger alarm on CH0
#define AMC7812_EALR_1      13   // CH 1 alarm bit, enable to trigger alarm on CH1
#define AMC7812_EALR_2      12   // CH 2 alarm bit, enable to trigger alarm on CH2
#define AMC7812_EALR_3      11   // CH 3 alarm bit, enable to trigger alarm on CH3
#define AMC7812_EALR_LT_L   10   // Chip low temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_LT_H    9   // Chip high temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D1_L    8   // D1 low temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D1_H    7   // D1 high temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D2_L    6   // D2 low temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D2_H    5   // D2 high temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D1_F    4   // D1 fail temp alarm bit, enable to trigger alarm
#define AMC7812_EALR_D2_F    3   // D2 fail temp alarm bit, enable to trigger alarm
#define AMC7812_ALR_LATCH    2   // 0: latched alarm, 1: non-latched alarm
// ADC CHANNEL REGISTER 0
// specify which channels are read ch0-12
// and if ch0-3 are single-ended or differential
// TODO: complete, pg 71
// ADC CHANNEL REGISTER 1
// specify which channels are read ch13-15
// TODO: complete, pg 72
// ADC GAIN REGISTER
// TODO: complete, pg 72
// AUTO-DAC-CLR-SOURCE REGISTER
// which alarms force the dac outputs to clear
// TODO: complete, pg 73
// AUTO-DAC-CLR-EN REGISTER
// TODO: complete, pg 74
// SW-DAC-CLR REGISTER
// software force dac clears
// TODO: complete, pg 74
// HW-DAC-CLR-EN-0 REGISTER
// hardware force dac clears on dac-clr-0 pin
// TODO: complete, pg 74
// HW-DAC-CLR-EN1 REGISTER
// hardware force dac clears on dac-clr-1 pin
// TODO: complete, pg 75
// DAC CONF REGISTER
// TODO: complete, pg 75
// DAC GAIN REGISTER
// TODO: complete, pg 75

// start reading again at pg 78

// STATUS REGISTER BIT MAPPINGS
// reading the status register clears alarms
// read-only
#define AMC7812_ALR_0       14  // channel 0 out-of-range
#define AMC7812_ALR_1       13  // channel 1 out-of-range
#define AMC7812_ALR_2       12  // channel 2 out-of-range
#define AMC7812_ALR_3       11  // channel 3 out-of-range
#define AMC7812_ALR_LT_L    10  // chip temp under range
#define AMC7812_ALR_LT_H     9  // chip temp over range
#define AMC7812_ALR_D1_L     8  // D1 temp under range
#define AMC7812_ALR_D1_H     7  // D1 temp over range
#define AMC7812_ALR_D2_L     6  // D2 temp under range
#define AMC7812_ALR_D2_H     5  // D2 temp over range
#define AMC7812_ALR_D1_F     4  // D1 fail temp alarm bit, enable to trigger alarm
#define AMC7812_ALR_D2_F     3  // D2 fail temp alarm bit, enable to trigger alarm
#define AMC7812_ALR_THERM    2  // Chip thermal alarm, T(chip) > 150 C


#define AMC7812_READ_MASK  0x80 // mask for read operations on addr byte
#define AMC7812_WRITE_MASK 0x00 // mask for write operations on addr byte

// functions
#include <stdint.h>

uint8_t  amc7812_Init ();
uint16_t amc7812_Read ( uint8_t addr );
uint16_t amc7812_Write( uint8_t addr, uint16_t value );
//=============================================================================
// temperature functions
uint16_t  amc7812_ReadTemp ( uint8_t sensor );
//=============================================================================
// adc functions
uint16_t  amc7812_ReadADC ( uint8_t n );
uint16_t  amc7812_EnableADC ( uint8_t n );
uint16_t  amc7812_EnableADCs ();
uint16_t  amc7812_DisableADCs ();
//=============================================================================
// dac functions
uint16_t  amc7812_ReadDAC  ( uint8_t n );
uint16_t  amc7812_WriteDAC ( uint8_t n, uint16_t value );
// dac gain is a register with one bit per channel,
// Gain(0) = 2*Vref
// Gain(1) = 5*Vref
uint16_t  amc7812_ReadDACGain();
// update all channel simultaneously
uint16_t  amc7812_WriteDACGain ( uint16_t value );
//=============================================================================
// configuration registers functions
uint8_t  amc7812_WriteAMCConfig  ( uint8_t n, uint16_t config );
uint16_t  amc7812_TriggerADCs ();

#endif
