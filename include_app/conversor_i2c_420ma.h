/*
 * conversor_i2c_420ma.h
 *
 * Created: 23/10/2020 11:23:46
 *  Author: Pablo
 */ 


#ifndef CONVERSOR_I2C_420MA_H_
#define CONVERSOR_I2C_420MA_H_

#define VERSION "R001 @ 2020-10-29" 

#define F_CPU 24000000

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../i2c/i2c_simple_master.h"

#define BPS120

int8_t CLKCTRL_init(void);

#define MAX_COMMAND_LEN	12

#define APAGAR_LED() ( PORTD.OUT |= PIN7_bm )
#define PRENDER_LED() ( PORTD.OUT &= ~PIN7_bm )

void reset(void);
int8_t WDT_init();
void CONFIG_led(void);
void cls(void);

void USART0_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);
char USART0_readChar(void);

int xprintf( const char *fmt, ...);

void DAC0_init(void);
void VREF_init(void);
void DAC0_setVal(uint16_t value);

void BPS120_init(void);
bool bps120_read( float *presion, bool debug );
void test_bps120(void);

void test_led(void);
void test_uart(void);
void test_dac(void);

void I2C_init(void);
void I2C_reset(bool debug );
int8_t I2C_read( bool debug, uint8_t i2c_dev_address, uint16_t i2c_rdStartAddress, uint8_t dataLength, char *rdBuffer );

void menu(void);

#endif /* CONVERSOR_I2C_420MA_H_ */