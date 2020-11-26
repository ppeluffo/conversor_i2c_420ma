/*
 * conversor_i2c_420ma.h
 *
 * Created: 23/10/2020 11:23:46
 *  Author: Pablo
 */ 


#ifndef CONVERSOR_I2C_420MA_H_
#define CONVERSOR_I2C_420MA_H_

#define F_CPU 24000000

#define FW_VERSION	"R001"
#define FW_FECHA	"2020/11/25"

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../i2c/i2c_simple_master.h"
#include "../include_os/nvm.h"

#define MAX_COMMAND_LEN	16

#define ESC 0x1B

int8_t CLKCTRL_init(void);

void reset(void);
int8_t WDT_init();
void cls(void);

void LED_init(void);
void led_flash(void);
void led_test(void);

void USART0_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);
char USART0_readChar(bool echo);
bool USART0_getChar( char *c );
int xprintf( const char *fmt, ...);
void uart_test(void);

void DAC0_init(void);
void VREF_init(void);
void DAC0_setVal(uint16_t value);
void dac_test(void);

void I2C_init(void);
void I2C_reset(void);
int8_t I2C_read( uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer );
int I2C_master_read_NI  ( const uint8_t i2c_dev_address, const uint16_t i2c_rdStartAddress, const uint8_t i2c_rdStartAddressLength, char *rdBuffer, size_t xBytes );

#define PSI_2_CMS	70.31

void BPS120_init(void);
bool bps120_read( float *presion, uint16_t *counts );
void bps120_test(void);
void bps120_calibrar(void);

#define HMAX_CMS_BPS120		70.31
#define PSI_MAX_BPS120		1.0

void NPA730_init(void);
bool npa730_read( float *presion );
void npa730_test(void);
#define HMAX_CMS_NPA730		1054.63
#define PSI_MAX_NPA730		15.0

void MAX30205_init(void);
bool max30205_read( float *presion );
void max30205_test(void);

void IDSENSOR_init(void);
uint8_t idsensor_read(void);
void idsensor_test(void);

bool mag2current(float mag4, float mag20, float mag );
void config_default(void);

typedef enum { DEBUG_LEVEL_OFF= 0x0, DEBUG_LEVEL_LOW = 0x02, DEBUG_LEVEL_MED = 0x06, DEBUG_LEVEL_HIGH = 0x0E } t_debuglevel;
typedef enum { DMASK_LOW = 0x02, DMASK_MED = 0x04, DMASK_HIGH = 0x08 } t_debugmasks;
	
#define SENSOR_BPS120	0
#define SENSOR_NPA730	1
#define SENSOR_MAX30205	2

//uint8_t sensor_id;
//t_debuglevel debug_level;
	
uint8_t sensor_id;
t_debuglevel debug_level;
uint16_t bps120_offset;

#define DEBUG_LOW   ( debug_level & DMASK_LOW )
#define DEBUG_MED   ( debug_level & DMASK_MED )
#define DEBUG_HIGH  ( debug_level & DMASK_HIGH )

typedef enum { CMDC_HELP = 1, CMDC_MEDIR, CMDC_LED, CMDC_CLS, CMDC_RESET, CMDC_DEBUG, CMDC_CALIBRAR, CMDC_DAC } t_command_codes;

#endif /* CONVERSOR_I2C_420MA_H_ */